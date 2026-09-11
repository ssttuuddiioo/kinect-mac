/* Exit quietly on a fatal signal - for the tracker child process only.
 *
 * MediaPipe 1.0.1 aborts every few minutes on macOS (its GPU pixel-buffer
 * allocator fails; see REVIEW.md). The tracker child is disposable - the app
 * restarts it - but a process killed by a signal gets a "Python quit
 * unexpectedly" dialog and a crash report, because Homebrew's Python runs
 * from a Python.app bundle. Exiting via _exit() instead is a normal exit, so
 * no dialog. The exit status still says what happened (128 + signal).
 *
 * Has to be native: a Python signal handler only sets a flag and returns, and
 * abort() then re-raises SIGABRT with the default action anyway.
 *
 * Never install this in the app itself: its crashes should be seen.
 */
#include <signal.h>
#include <string.h>
#include <unistd.h>

static void on_fatal(int sig) {
    /* async-signal-safe only: write(2) and _exit(2) */
    static const char msg[] =
        "tracker child: MediaPipe hit a fatal signal - exiting for a restart\n";
    (void)!write(2, msg, sizeof msg - 1);
    _exit(128 + sig);
}

int crashguard_install(void) {
    static const int sigs[] = { SIGABRT, SIGSEGV, SIGBUS, SIGILL, SIGFPE };
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_fatal;
    sigemptyset(&sa.sa_mask);
    int failed = 0;
    for (unsigned i = 0; i < sizeof sigs / sizeof sigs[0]; ++i)
        failed |= sigaction(sigs[i], &sa, NULL);
    return failed ? -1 : 0;
}
