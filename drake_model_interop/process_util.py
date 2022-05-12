from functools import partial
import os
import select
import shlex
import signal
import sys
from subprocess import Popen, PIPE, STDOUT


def shlex_join(args):
    # TODO(eric.cousineau): Replace this with `shlex.join` when we exclusively
    # use Python>=3.8.
    return " ".join(map(shlex.quote, args))


def signal_processes(process_list, sig=signal.SIGINT, block=True):
    """
    Robustly sends a singal to processes that are still alive. Ignores status
    codes.

    @param process_list List[Popen] Processes to ensure are sent a signal.
    @param sig Signal to send. Default is `SIGINT1.
    @param block Block until process exits.
    """
    for process in process_list:
        if process.poll() is None:
            process.send_signal(sig)
    if block:
        for process in process_list:
            if process.poll() is None:
                process.wait()


def read_available(f, timeout=0.0, chunk_size=1024, empty=None):
    """
    Reads all available data on a given file. Useful for using PIPE with Popen.

    @param timeout Timeout for `select`.
    @param chunk_size How much to try and read.
    @param empty Starting point / empty value. Default value is "" (good for
    strings).
    """
    readable, _, _ = select.select([f], [], [f], timeout)
    if empty is None:
        empty = bytes()
    out = empty
    if f in readable:
        while True:
            cur = os.read(f.fileno(), chunk_size)
            out += cur
            if len(cur) < chunk_size:
                break
    return out


def print_prefixed(text, prefix="", streams=[sys.stdout]):
    """
    Prints non-empty text to a list of streams with a prefix.
    """
    if text.endswith("\n"):
        text = text[:-1]
    if not streams or not text:
        return
    prefixed = "\n".join([prefix + line for line in text.split("\n")])
    for stream in streams:
        stream.write(prefixed + "\n")


def bind_print_prefixed(prefix):
    return partial(print_prefixed, prefix=prefix)


class StreamCollector(object):
    """
    Collects available text from a stream (e.g. a PIPE from Popen).

    @param on_new_text When new text is received; can use
    `print_prefixed`.  If this is iterable, each item will
    be called with the newly received text.

    @param on_new_text Callback functor(s) to be invoked when new text
    is received. Can be a scalar (for a single functor) or an iterable
    (for multiple functors, all of which will be called for all new
    text). See `print_prefixed` above for an example functor
    implementation.
    """

    def __init__(self, streams, on_new_text=tuple()):
        self._streams = streams
        self._text = ""
        try:
            # Check if on_new_text is iterable
            iter(on_new_text)
            self._on_new_text = on_new_text
        except TypeError:
            self._on_new_text = (on_new_text,)

    def clear(self):
        self._text = ""

    def get_text(self, timeout=0.0):
        """
        Gets current text.
        @param timeout Timeout for polling each stream. If None, will not
        update.
        """
        if timeout is not None:
            # Get available bytes from streams.
            text_new = ""
            for stream in self._streams:
                if stream.closed:
                    continue
                data = read_available(stream, timeout=timeout)
                if isinstance(data, bytes):
                    # HACK
                    data = data.decode("utf-8")
                text_new += data
            if text_new:
                for on_new_text in self._on_new_text:
                    on_new_text(text_new)
            self._text += text_new
        return self._text


class CapturedProcess(object):
    """
    Captures a process's stdout and stderr to a StreamCollector and provides
    (non-blocking) polling access to the latest output.
    Useful for interacting with input / output in a simple fashion.

    Note: This has only been designed for simple offline scripts, not for
    controlling realtime stuff.
    For complex state machine process interaction, use `pexpect`.
    """

    def __init__(
        self, args, stderr=STDOUT, on_new_text=None, simple_encoding=True, **kwargs
    ):
        # Python processes don't like buffering by default.
        args = ["env", "PYTHONUNBUFFERED=1", "stdbuf", "-o0"] + args
        self._args = args
        if simple_encoding:
            kwargs.update(encoding="utf8", universal_newlines=True, bufsize=1)
        proc = Popen(args, stdin=PIPE, stdout=PIPE, stderr=stderr, **kwargs)
        self.proc = proc
        streams = [proc.stdout]
        if proc.stderr:  # `None` if `stderr=STDOUT` is specified via `Popen`
            streams += [proc.stderr]
        self.output = StreamCollector(streams, on_new_text=on_new_text)

    def __repr__(self):
        return "<CapturedProcess {}>".format(self._args)

    def poll(self):
        """Polls process, returning exit code."""
        self.output.get_text()  # Flush text for debugging.
        return self.proc.poll()

    def wait(self):
        """Waits until a process ends, polling the process (ensuring that
        `on_new_text` is called). Returns exit status."""
        while self.poll() is None:
            pass
        return self.poll()

    def close(self):
        """Attempts to signal process to close. If process is already closed,
        this is a no-op."""
        signal_processes([self.proc])
