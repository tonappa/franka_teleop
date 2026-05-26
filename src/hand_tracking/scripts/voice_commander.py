#!/usr/bin/env python3
"""Voice commander node for Panda teleoperation.

Listens for the wake word "Panda" using dual-language Vosk models
(English + Italian), then waits for an explicit engage/disengage
command to control the clutch via /bridge/clutch.

Uses PulseAudio (pactl/parec) directly for audio capture, bypassing
PortAudio's limited backend support inside Docker.
"""

import json
import math
import struct
import subprocess
import sys
import threading
import time

import rospy
from std_msgs.msg import Bool, Empty
from vosk import Model, KaldiRecognizer, SetLogLevel

# ── Command vocabularies ─────────────────────────────────────────────
# Rules: no overlaps between groups, avoid short/common words (poor Vosk
# accuracy), prefer 2+ syllable words that are phonetically distinct.

ENGAGE_WORDS = {
    # English
    "stop", "freeze", "hold",
    # Italian
    "ferma", "fermati", "blocca",
}

DISENGAGE_WORDS = {
    # English
    "start", "follow",
    # Italian
    "vai", "seguimi", "parti",
}

RESET_WORDS = {
    # English
    "reset", "home",
    # Italian
    "resetta", "casa",
}

OPEN_GRIPPER_WORDS = {
    # English
    "open", "release", "drop",
    # Italian
    "apri", "rilascia", "molla",
}

CLOSE_GRIPPER_WORDS = {
    # English
    "close", "grab", "grasp", "catch",
    # Italian
    "chiudi", "afferra", "prendi",
}

LOCK_ORIENTATION_WORDS = {
    # English
    "rigid", "steady", "fixed",
    # Italian
    "rigido", "fisso", "saldo",
}

UNLOCK_ORIENTATION_WORDS = {
    # English
    "rotate", "smooth", "fluid",
    # Italian
    "ruota", "fluido", "sciolto",
}

RESET_ORIENTATION_WORDS = {
    # English
    "straight", "align", "level",
    # Italian
    "dritto", "allinea", "livella","addrizza"
}

WAKE_WORD = "panda"

# Vosk recogniser sample rate
SAMPLE_RATE = 16000
BLOCK_SIZE = 2000          # 2000 bytes = 1000 samples ≈ 62 ms at 16 kHz mono s16le
COOLDOWN_SEC = 1.5         # ignore duplicate commands for this long


# ── PulseAudio microphone selection ───────────────────────────────────
def list_pa_sources():
    """Return a list of (source_name, description) for PulseAudio sources."""
    try:
        out = subprocess.check_output(
            ["pactl", "list", "sources", "short"],
            text=True, stderr=subprocess.DEVNULL)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return []
    sources = []
    for line in out.strip().splitlines():
        parts = line.split("\t")
        if len(parts) >= 2:
            name = parts[1]
            # Skip monitor (loopback) and output-only sinks
            if ".monitor" in name:
                continue
            sources.append(name)
    if not sources:
        return []

    # Get human-readable descriptions
    result = []
    for name in sources:
        try:
            props = subprocess.check_output(
                ["pactl", "list", "sources"], text=True, stderr=subprocess.DEVNULL)
            desc = name  # fallback
            in_target = False
            for pline in props.splitlines():
                pline = pline.strip()
                if pline.startswith("Name:") and name in pline:
                    in_target = True
                elif pline.startswith("Name:"):
                    in_target = False
                elif in_target and pline.startswith("Description:"):
                    desc = pline.split(":", 1)[1].strip()
                    break
            result.append((name, desc))
        except (subprocess.CalledProcessError, FileNotFoundError):
            result.append((name, name))
    return result


def select_microphone(param_index):
    """Return the PulseAudio source name to use for recording.

    If *param_index* >= 0, use it as a 1-based index into the source
    list.  Otherwise print an interactive menu and ask the user.
    """
    sources = list_pa_sources()

    if not sources:
        rospy.logfatal("No PulseAudio audio sources found! "
                       "Is PulseAudio forwarded into the container?")
        sys.exit(1)

    if param_index >= 0:
        # Treat as 1-based index
        if param_index < 1 or param_index > len(sources):
            rospy.logfatal(
                "mic_device=%d is out of range. Valid: 1-%d",
                param_index, len(sources))
            sys.exit(1)
        name, desc = sources[param_index - 1]
        rospy.loginfo("Using mic %d: %s", param_index, desc)
        return name

    # Interactive prompt (1-indexed)
    print("\n── Available Input Devices ─────────────────")
    for pos, (name, desc) in enumerate(sources, start=1):
        print(f"  {pos}) {desc}")
    print("────────────────────────────────────────────")

    while True:
        try:
            choice = int(input("Select microphone [1-%d]: " % len(sources)))
            if 1 <= choice <= len(sources):
                return sources[choice - 1][0]
        except (ValueError, EOFError):
            pass
        print("Invalid selection, try again.")


# ── Helpers ───────────────────────────────────────────────────────────
def words_from_json(json_str):
    """Extract the list of recognised words from a Vosk JSON result."""
    try:
        data = json.loads(json_str)
    except json.JSONDecodeError:
        return []
    # "text" key for final results, "partial" for partials
    text = data.get("text", "") or data.get("partial", "")
    return text.lower().split()


def _generate_tones(tones, volume=0.5):
    """Pre-generate raw s16le bytes for a sequence of (freq_hz, duration_s) tones."""
    samples = bytearray()
    for freq, dur in tones:
        n = int(SAMPLE_RATE * dur)
        for i in range(n):
            t = i / SAMPLE_RATE
            fade = min(i / (SAMPLE_RATE * 0.008),
                       (n - 1 - i) / (SAMPLE_RATE * 0.008), 1.0)
            val = int(volume * fade * 32767
                      * math.sin(2 * math.pi * freq * t))
            samples += struct.pack("<h", val)
    return bytes(samples)


# Pre-computed audio buffers (generated once at import time)
_LISTEN_BEEP = _generate_tones([(660, 0.10), (1040, 0.12)])
_CONFIRM_BEEP = _generate_tones([(1040, 0.08), (0, 0.03), (1040, 0.08)])
_TIMEOUT_BEEP = _generate_tones([(440, 0.22)])


def _play_raw(raw_audio):
    """Send pre-computed raw s16le audio to paplay in a background thread."""
    def _play():
        try:
            p = subprocess.Popen(
                ["paplay", "--raw", "--format=s16le",
                 "--rate=%d" % SAMPLE_RATE, "--channels=1"],
                stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
            p.communicate(input=raw_audio)
        except Exception:
            pass  # non-critical — don't crash if audio playback fails
    threading.Thread(target=_play, daemon=True).start()


def play_listen_beep():
    """Ascending chirp — signals 'listening for command'."""
    _play_raw(_LISTEN_BEEP)


def play_confirm_beep():
    """Double high-tone chirp — signals 'command accepted'."""
    _play_raw(_CONFIRM_BEEP)


def play_timeout_beep():
    """Single low tone — signals 'gave up listening'."""
    _play_raw(_TIMEOUT_BEEP)


def check_command(words):
    """Return (topic, msg, label) if any command word is found, else None.

    topic is "clutch", "reset", or "gripper".
    """
    for w in words:
        if w in ENGAGE_WORDS:
            return "clutch", Bool(data=True), w
        if w in DISENGAGE_WORDS:
            return "clutch", Bool(data=False), w
        if w in RESET_WORDS:
            return "reset", Empty(), w
        if w in OPEN_GRIPPER_WORDS:
            return "gripper", Bool(data=False), w
        if w in CLOSE_GRIPPER_WORDS:
            return "gripper", Bool(data=True), w
        if w in LOCK_ORIENTATION_WORDS:
            return "lock_orientation", Bool(data=True), w
        if w in UNLOCK_ORIENTATION_WORDS:
            return "lock_orientation", Bool(data=False), w
        if w in RESET_ORIENTATION_WORDS:
            return "reset_orientation", Empty(), w
    return None


def _dispatch(topic, msg, clutch_pub, reset_pub, gripper_pub, lock_orient_pub,
              reset_orient_pub):
    """Publish *msg* on the right publisher and return a log label."""
    if topic == "clutch":
        clutch_pub.publish(msg)
        return "ENGAGE" if msg.data else "DISENGAGE"
    if topic == "reset":
        reset_pub.publish(msg)
        return "RESET"
    if topic == "gripper":
        gripper_pub.publish(msg)
        return "GRIPPER CLOSE" if msg.data else "GRIPPER OPEN"
    if topic == "lock_orientation":
        lock_orient_pub.publish(msg)
        return "LOCK ORIENTATION" if msg.data else "UNLOCK ORIENTATION"
    if topic == "reset_orientation":
        reset_orient_pub.publish(msg)
        return "RESET ORIENTATION"
    return topic.upper()


# ── Main loop ─────────────────────────────────────────────────────────
def main():
    rospy.init_node("voice_commander")
    SetLogLevel(-1)  # suppress Vosk debug spam

    # Parameters
    en_model_path = rospy.get_param("~en_model_path",
                                    "/opt/vosk-models/vosk-model-small-en-us")
    it_model_path = rospy.get_param("~it_model_path",
                                    "/opt/vosk-models/vosk-model-small-it")
    mic_device = rospy.get_param("~mic_device", -1)
    wake_timeout = rospy.get_param("~wake_timeout", 4.0)
    debug = rospy.get_param("~debug", False)

    # Publishers
    clutch_pub = rospy.Publisher("/bridge/clutch", Bool, queue_size=1)
    reset_pub = rospy.Publisher("/bridge/reset", Empty, queue_size=1)
    gripper_pub = rospy.Publisher("/gripper/command", Bool, queue_size=1)
    lock_orient_pub = rospy.Publisher("/bridge/lock_orientation", Bool, queue_size=1)
    reset_orient_pub = rospy.Publisher("/bridge/reset_orientation", Empty, queue_size=1)

    # Microphone
    source_name = select_microphone(mic_device)

    # Load models
    rospy.loginfo("Loading English model from %s ...", en_model_path)
    model_en = Model(en_model_path)
    rospy.loginfo("Loading Italian model from %s ...", it_model_path)
    model_it = Model(it_model_path)

    rec_en = KaldiRecognizer(model_en, SAMPLE_RATE)
    rec_it = KaldiRecognizer(model_it, SAMPLE_RATE)
    rec_en.SetWords(True)
    rec_it.SetWords(True)

    rospy.loginfo("Voice commander ready. Say '%s' to activate.", WAKE_WORD)

    # Audio capture command
    parec_cmd = [
        "parec",
        "--device=" + source_name,
        "--format=s16le",
        "--rate=%d" % SAMPLE_RATE,
        "--channels=1",
    ]

    # Persistent worker thread for Italian recogniser
    _it_in = threading.Event()     # main -> worker: "audio_bytes is ready"
    _it_done = threading.Event()   # worker -> main: "result is ready"
    _it_buf = [None, None]         # [audio_bytes, final_flag]

    def _it_worker():
        while not rospy.is_shutdown():
            _it_in.wait()
            _it_in.clear()
            _it_buf[1] = rec_it.AcceptWaveform(_it_buf[0])
            _it_done.set()

    threading.Thread(target=_it_worker, daemon=True).start()

    # State
    state = "IDLE"        # IDLE | LISTENING
    wake_time = 0.0
    cooldown_until = 0.0
    proc = None

    while not rospy.is_shutdown():
        # (Re)start parec if needed
        if proc is None or proc.poll() is not None:
            if proc is not None:
                rospy.logwarn("Audio stream lost — reconnecting in 1 s ...")
                time.sleep(1.0)
            proc = subprocess.Popen(parec_cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL)
            rospy.loginfo("Audio capture started.")
            state = "IDLE"
            rec_en.Reset()
            rec_it.Reset()

        audio_bytes = proc.stdout.read(BLOCK_SIZE)
        if not audio_bytes:
            # parec died — loop back to reconnect
            continue
        now = time.time()

        # Feed both recognisers in parallel
        _it_buf[0] = audio_bytes
        _it_done.clear()
        _it_in.set()
        en_final = rec_en.AcceptWaveform(audio_bytes)
        _it_done.wait()
        it_final = _it_buf[1]

        # Collect words from both (partials + finals)
        all_words = []
        if en_final:
            all_words += words_from_json(rec_en.Result())
        else:
            all_words += words_from_json(rec_en.PartialResult())
        if it_final:
            all_words += words_from_json(rec_it.Result())
        else:
            all_words += words_from_json(rec_it.PartialResult())

        if not all_words:
            # Check for listening timeout
            if state == "LISTENING" and now - wake_time > wake_timeout:
                play_timeout_beep()
                rospy.logwarn("Wake timeout — no command heard.")
                state = "IDLE"
            continue

        # ── State machine ──
        if state == "IDLE":
            if WAKE_WORD in all_words:
                play_listen_beep()
                rospy.loginfo("Wake word detected — listening ...")
                state = "LISTENING"
                wake_time = now
                # Don't reset recognisers — the command word may
                # already be buffered in the current partial result.

        if state == "LISTENING":
            if now - wake_time > wake_timeout:
                play_timeout_beep()
                rospy.logwarn("Wake timeout — no command heard.")
                state = "IDLE"
                continue

            if now < cooldown_until:
                continue

            if debug and all_words:
                rospy.logdebug("Heard: %s", " ".join(all_words))

            result = check_command(all_words)
            if result:
                topic, msg, word = result
                play_confirm_beep()
                rospy.loginfo("Command '%s' → %s",
                              word, _dispatch(topic, msg,
                                               clutch_pub, reset_pub,
                                               gripper_pub, lock_orient_pub,
                                               reset_orient_pub))
                cooldown_until = now + COOLDOWN_SEC
                state = "IDLE"
                rec_en.Reset()
                rec_it.Reset()

    if proc is not None:
        proc.terminate()
        proc.wait()


if __name__ == "__main__":
    main()
