// The connection never replays stored demands or claims control on open.
function connectRobot(WebSocketClass, location, setStatus, resetDemand, environment = globalThis) {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const socket = new WebSocketClass(`${protocol}//${location.host}/ws`);
    let lastError = '';
    let demand = { left: 0, right: 0 };
    let timer = null;
    let lastSent = 0;
    let focused = environment.document?.hasFocus?.() ?? true;
    const inputs = new Set();
    const now = () => environment.performance.now();
    const available = () => focused && !environment.document?.hidden;

    function clearIntent() {
        if (timer !== null) environment.clearInterval(timer);
        timer = null;
        demand = { left: 0, right: 0 };
        inputs.clear();
        resetDemand();
    }

    function send(left, right) {
        if (socket.readyState !== WebSocketClass.OPEN) return false;
        socket.send(JSON.stringify({ left, right }));
        lastSent = now();
        return true;
    }

    function suspend() {
        const moving = demand.left !== 0 || demand.right !== 0;
        clearIntent();
        if (moving) send(0, 0);
    }

    function sendDriveCommand(left, right) {
        if (!available() || socket.readyState !== WebSocketClass.OPEN) return false;
        if (!Number.isFinite(left) || !Number.isFinite(right)) return false;
        demand = { left, right };
        if (!send(left, right)) return false;
        if (left === 0 && right === 0) {
            if (timer !== null) environment.clearInterval(timer);
            timer = null;
        } else if (timer === null) {
            timer = environment.setInterval(() => {
                if (!available()) { suspend(); return; }
                // A suspended/throttled event loop must not resurrect old input.
                if (now() - lastSent >= 500) {
                    clearIntent();
                    lastError = 'Command renewal paused too long; reconnect with fresh input.';
                    console.error(`[robot-control] expired: ${lastError}`);
                    socket.close();
                    return;
                }
                if (!send(demand.left, demand.right)) clearIntent();
            }, 100);
        }
        return true;
    }

    environment.addEventListener?.('blur', () => { focused = false; suspend(); });
    environment.addEventListener?.('focus', () => { focused = true; });
    environment.addEventListener?.('pagehide', suspend);
    environment.document?.addEventListener('visibilitychange', () => {
        if (environment.document.hidden) suspend();
        else focused = environment.document.hasFocus?.() ?? focused;
    });

    socket.onopen = () => {
        clearIntent();
        setStatus('Connected — move a joystick to request control.');
    };
    socket.onmessage = (event) => {
        if (socket.readyState !== WebSocketClass.OPEN) return;
        let message;
        try {
            message = JSON.parse(event.data);
            if (!message || typeof message !== 'object') throw new Error('expected an object');
        } catch (error) {
            console.error('[robot-control] Invalid server response:', error);
            clearIntent();
            lastError = 'Invalid server response.';
            setStatus(lastError);
            socket.close();
            return;
        }
        if (message.status === 'accepted') {
            lastError = '';
            setStatus('You control the robot. Close this tab to release control.');
        } else if (message.code && message.code !== 'superseded') {
            console.error(`[robot-control] ${message.code}: ${message.error}`);
            clearIntent();
            lastError = message.code === 'busy'
                ? 'Another client controls the robot or is stopping. Try again after it releases control.'
                : message.error;
            setStatus(lastError);
            // A rejected command is never retried. On overload/fault, close
            // our stream so any ownership is released and smooth stopping begins.
            if (message.code !== 'busy') socket.close();
        }
    };
    socket.onerror = (event) => {
        console.error('[robot-control] WebSocket connection error:', event);
        clearIntent();
        setStatus('Connection error.');
    };
    socket.onclose = () => {
        clearIntent();
        setStatus(`${lastError ? lastError + ' ' : ''}Disconnected — reload to reconnect.`);
    };
    return {
        sendDriveCommand,
        startInput(side) {
            if (available() && socket.readyState === WebSocketClass.OPEN) inputs.add(side);
        },
        moveInput(side, value) {
            if (!inputs.has(side)) return false;
            return sendDriveCommand(side === 'left' ? value : demand.left,
                                    side === 'right' ? value : demand.right);
        },
        endInput(side) {
            if (!inputs.delete(side)) return false;
            return sendDriveCommand(side === 'left' ? 0 : demand.left,
                                    side === 'right' ? 0 : demand.right);
        },
    };
}

if (typeof module !== 'undefined') module.exports = { connectRobot };
