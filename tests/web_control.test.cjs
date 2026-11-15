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
    const connection = connectRobot(WebSocket, { protocol, host: 'robot.local:8080' },
        value => { status = value; }, () => { resets++; });
    return { connection, socket, status: () => status, resets: () => resets };
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
