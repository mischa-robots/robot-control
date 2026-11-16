const test = require('node:test');
const assert = require('node:assert/strict');
const { connectRobot } = require('../static/control.js');

function fixture(protocol = 'http:') {
    let socket;
    class WebSocket {
        static OPEN = 1;
        constructor(url) { this.url = url; this.readyState = 0; this.sent = []; socket = this; }
        send(message) { this.sent.push(JSON.parse(message)); }
        close() { this.readyState = 3; this.onclose(); }
    }
    let status;
    let resets = 0;
    let clock = 0;
    let nextId = 0;
    const timers = new Map();
    const listeners = {};
    const environment = {
        performance: { now: () => clock },
        document: { hidden: false, hasFocus: () => true,
            addEventListener: (name, callback) => { listeners[name] = callback; } },
        addEventListener: (name, callback) => { listeners[name] = callback; },
        setInterval(callback, period) { timers.set(++nextId, { callback, period, next: clock + period }); return nextId; },
        clearInterval(id) { timers.delete(id); },
    };
    function advance(ms) {
        const end = clock + ms;
        while (true) {
            const next = [...timers.values()].sort((a, b) => a.next - b.next)[0];
            if (!next || next.next > end) break;
            clock = next.next;
            next.next += next.period;
            next.callback();
        }
        clock = end;
    }
    function stall(ms) {
        clock += ms;
        for (const timer of [...timers.values()]) timer.callback();
    }
    const connection = connectRobot(WebSocket, { protocol, host: 'robot.local:8080' },
        value => { status = value; }, () => { resets++; }, environment);
    return { connection, socket, status: () => status, resets: () => resets, advance, stall, listeners, environment, timers };
}

test('URL preserves scheme/host/port; no disconnected sends or automatic demand replay', (t) => {
    const errors = t.mock.method(console, 'error', () => {});
    for (const protocol of ['http:', 'https:']) {
        const f = fixture(protocol);
        assert.equal(f.socket.url, `${protocol === 'https:' ? 'wss:' : 'ws:'}//robot.local:8080/ws`);
        assert.equal(f.connection.sendDriveCommand(0.5, 0.5), false);
        f.socket.readyState = 1;
        f.socket.onopen();
        assert.equal(f.socket.sent.length, 0);
        f.connection.sendDriveCommand(0.2, -0.3);
        assert.deepEqual(f.socket.sent, [{ left: 0.2, right: -0.3 }]);
        f.socket.close();
        assert.equal(f.connection.sendDriveCommand(0.5, 0.5), false);
        f.socket.readyState = 1;
        f.socket.onopen();
        assert.equal(f.socket.sent.length, 1);
    }
    assert.equal(errors.mock.callCount(), 0);
});

test('busy resets local demands and never retries; acceptance reports ownership', (t) => {
    const errors = t.mock.method(console, 'error', () => {});
    const f = fixture();
    f.socket.readyState = 1;
    f.socket.onmessage({ data: JSON.stringify({ code: 'busy', error: 'busy' }) });
    assert.match(f.status(), /Another client/);
    assert.equal(f.resets(), 1);
    assert.equal(f.socket.sent.length, 0);
    assert.equal(f.socket.readyState, 1);
    f.socket.onmessage({ data: JSON.stringify({ status: 'accepted' }) });
    assert.match(f.status(), /You control/);
    assert.equal(errors.mock.callCount(), 1);
    assert.equal(errors.mock.calls[0].arguments[0], '[robot-control] busy: busy');
});

test('overload and fault close the stream instead of retrying stale commands', (t) => {
    const errors = t.mock.method(console, 'error', () => {});
    for (const code of ['queue_full', 'fault', 'shutdown']) {
        const f = fixture();
        f.socket.readyState = 1;
        f.socket.onmessage({ data: JSON.stringify({ code, error: 'Rejected command' }) });
        assert.equal(f.socket.readyState, 3);
        assert.match(f.status(), /Rejected command.*Disconnected/);
        assert.equal(f.socket.sent.length, 0);
    }
    assert.deepEqual(errors.mock.calls.map(call => call.arguments[0]), [
        '[robot-control] queue_full: Rejected command',
        '[robot-control] fault: Rejected command',
        '[robot-control] shutdown: Rejected command',
    ]);
});

test('superseded commands stay quiet; malformed replies and connection errors are logged', (t) => {
    const errors = t.mock.method(console, 'error', () => {});
    const f = fixture();
    f.socket.readyState = 1;
    f.socket.onmessage({ data: JSON.stringify({ code: 'superseded', error: 'stopped' }) });
    assert.equal(f.socket.readyState, 1);
    assert.equal(errors.mock.callCount(), 0);
    f.socket.onmessage({ data: 'not json' });
    assert.equal(f.socket.readyState, 3);
    assert.match(f.status(), /Invalid server response/);
    assert.equal(errors.mock.callCount(), 1);
    assert.match(errors.mock.calls[0].arguments[0], /Invalid server response/);
    const event = { type: 'error' };
    f.socket.onerror(event);
    assert.equal(errors.mock.callCount(), 2);
    assert.equal(errors.mock.calls[1].arguments[1], event);
});

test('unchanged held input renews every 100 ms; release sends zero and ends renewal', () => {
    const f = fixture();
    f.socket.readyState = 1;
    f.socket.onopen();
    f.connection.startInput('left');
    f.connection.moveInput('left', 0.4);
    f.advance(1100);
    assert.equal(f.socket.sent.length, 12);
    assert.ok(f.socket.sent.every(value => value.left === 0.4 && value.right === 0));
    f.connection.endInput('left');
    assert.deepEqual(f.socket.sent.at(-1), { left: 0, right: 0 });
    const count = f.socket.sent.length;
    f.advance(1000);
    assert.equal(f.socket.sent.length, count);
    assert.equal(f.timers.size, 0);
});

test('hiding or blurring clears both axes; focus/visibility and old gestures do not resume', () => {
    for (const event of ['blur', 'visibilitychange', 'pagehide']) {
        const f = fixture();
        f.socket.readyState = 1;
        f.connection.startInput('left');
        f.connection.startInput('right');
        f.connection.moveInput('left', 0.4);
        f.connection.moveInput('right', 0.5);
        if (event === 'visibilitychange') f.environment.document.hidden = true;
        f.listeners[event]();
        assert.deepEqual(f.socket.sent.at(-1), { left: 0, right: 0 });
        const count = f.socket.sent.length;
        f.advance(1000);
        f.environment.document.hidden = false;
        f.listeners.visibilitychange();
        f.listeners.focus();
        f.connection.moveInput('left', 0.8);
        f.connection.endInput('right');
        f.advance(1000);
        assert.equal(f.socket.sent.length, count);
        f.connection.startInput('left');
        f.connection.moveInput('left', 0.2);
        assert.deepEqual(f.socket.sent.at(-1), { left: 0.2, right: 0 });
    }
});

test('a delayed heartbeat closes instead of replaying a stale nonzero demand', (t) => {
    t.mock.method(console, 'error', () => {});
    const f = fixture();
    f.socket.readyState = 1;
    f.connection.sendDriveCommand(0.4, 0.4);
    f.stall(500);
    assert.equal(f.socket.sent.length, 1);
    assert.equal(f.socket.readyState, 3);
    assert.equal(f.timers.size, 0);
    assert.match(f.status(), /renewal paused/);
});

test('busy, expiry and connection loss cancel renewal; acceptance cannot revive it', (t) => {
    t.mock.method(console, 'error', () => {});
    for (const code of ['busy', 'expired', 'connection']) {
        const f = fixture();
        f.socket.readyState = 1;
        f.connection.startInput('left');
        f.connection.moveInput('left', 0.4);
        if (code === 'connection') f.socket.onerror({ type: 'error' });
        else f.socket.onmessage({ data: JSON.stringify({ code, error: code }) });
        f.socket.onmessage({ data: JSON.stringify({ status: 'accepted' }) });
        f.advance(1000);
        assert.equal(f.socket.sent.length, 1);
        assert.equal(f.timers.size, 0);
        assert.equal(f.connection.moveInput('left', 0.8), false);
    }
});
