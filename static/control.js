// The connection never replays stored demands or claims control on open.
function connectRobot(WebSocketClass, location, setStatus, resetDemand) {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const socket = new WebSocketClass(`${protocol}//${location.host}/ws`);
    let lastError = '';

    socket.onopen = () => {
        resetDemand();
        setStatus('Connected — move a joystick to request control.');
    };
    socket.onmessage = (event) => {
        let message;
        try {
            message = JSON.parse(event.data);
        } catch (error) {
            console.error('[robot-control] Invalid server response:', error);
            resetDemand();
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
            resetDemand();
            lastError = message.code === 'busy'
                ? 'Another client controls the robot or is stopping. Try again after it releases control.'
                : message.error;
            setStatus(lastError);
            // A rejected command is never retried. On overload/fault, close
            // our stream so any ownership is released and ordinary stop runs.
            if (message.code !== 'busy') socket.close();
        }
    };
    socket.onerror = (event) => {
        console.error('[robot-control] WebSocket connection error:', event);
        resetDemand();
        setStatus('Connection error.');
    };
    socket.onclose = () => {
        resetDemand();
        setStatus(`${lastError ? lastError + ' ' : ''}Disconnected — reload to reconnect.`);
    };
    return {
        sendDriveCommand(left, right) {
            if (socket.readyState !== WebSocketClass.OPEN) return false;
            socket.send(JSON.stringify({ left, right }));
            return true;
        },
    };
}

if (typeof module !== 'undefined') module.exports = { connectRobot };
