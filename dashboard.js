document.addEventListener('DOMContentLoaded', (event) => {
    
    // --- Configuration and DOM Element Selection ---
    const statusDiv = document.getElementById('status');
    
    // 1. IMPORTANT: REPLACE WITH YOUR ACTIVE WSS:// TUNNEL FOR OBD (8080)
    const OBD_WEBSOCKET_URL = 'wss://orange-toys-ring.loca.lt'; 
    const socket = new WebSocket(OBD_WEBSOCKET_URL);

    // 2. IMPORTANT: REPLACE WITH YOUR ACTIVE WSS:// TUNNEL FOR CAMERA (8081)
    const CAMERA_WEBSOCKET_URL = 'wss://metal-animals-pay.loca.lt'; // REPLACE THIS
    let cameraSocket = null;
    let isCameraOn = false;

    // DOM references for metrics
    const metricsEls = {
        speed: document.getElementById('speed'),
        rpm: document.getElementById('rpm'),
        throttlePosition: document.getElementById('throttlePosition'),
        coolantTemp: document.getElementById('coolantTemp'),
        fuelLevel: document.getElementById('fuelLevel'),
        intakeAirTemp: document.getElementById('intakeAirTemp'),
        timingAdvance: document.getElementById('timingAdvance'),
        maf: document.getElementById('maf'),
        shortFuelTrim: document.getElementById('shortFuelTrim'),
        longFuelTrim: document.getElementById('longFuelTrim'),
        engineLoad: document.getElementById('engineLoad'),
        barometricPressure: document.getElementById('barometricPressure'),
        ambientAirTemp: document.getElementById('ambientAirTemp'),
        tirePressure: document.getElementById('tirePressure'),
        batteryVoltage: document.getElementById('batteryVoltage'),
        acceleration: document.getElementById('acceleration'),
        tripDistance: document.getElementById('tripDistance'),
        dtc: document.getElementById('dtc'),
        ignitionState: document.getElementById('ignitionState'),
    };

    // This must match the <img> element in index.html
    const videoElement = document.getElementById('cameraFeed'); 
    const cameraContainer = document.getElementById('camera-container');
    const cameraToggleButton = document.getElementById('camera-toggle-btn');
    const cameraToggleStatus = document.getElementById('camera-toggle-status');
    const cameraPlaceholder = document.getElementById('camera-placeholder');


    // ======================================================
    // ====================================================================
    //                       LIDAR (ROS/ROS2 via rosbridge)
    //   Subscribes to /lidar_points (sensor_msgs/PointCloud2) and renders
    //   a simple top-down 2D scatter on the #lidar-canvas.
    //   Works with ROS1 or ROS2 through rosbridge_server.
    //   Set window.ROSBRIDGE_URL to override (default ws://localhost:9090).
    // ====================================================================

    const lidarCanvas = document.getElementById('lidar-canvas');
    const lidarCtx = lidarCanvas.getContext('2d');
    const lidarStatus = document.getElementById('lidar-status');

    // Resize canvas to container
    function resizeLidarCanvas() {
        const rect = lidarCanvas.getBoundingClientRect();
        lidarCanvas.width = Math.max(320, Math.floor(rect.width));
        lidarCanvas.height = Math.max(180, Math.floor(rect.height));
    }
    window.addEventListener('resize', resizeLidarCanvas);
    resizeLidarCanvas();

    // Connect to rosbridge
    const ROSBRIDGE_URL = window.ROSBRIDGE_URL || 'ws://localhost:9090';
    let ros = null;
    let lidarTopic = null;
    let lastLidarPoints = 0;

    function connectRos() {
        try {
            ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
        } catch (e) {
            console.error("Failed to construct ROSLIB.Ros:", e);
            lidarStatus.textContent = "(ROSLIB error)";
            return;
        }

        ros.on('connection', function () {
            console.log("✅ Connected to rosbridge at", ROSBRIDGE_URL);
            lidarStatus.textContent = "(connected)";
            subscribeLidar();
        });

        ros.on('error', function (err) {
            console.error("Rosbridge error:", err);
            lidarStatus.textContent = "(error)";
        });

        ros.on('close', function () {
            console.warn("Rosbridge connection closed.");
            lidarStatus.textContent = "(disconnected)";
            // Reconnect after a short delay
            setTimeout(connectRos, 2000);
        });
    }

    function subscribeLidar() {
        // Try both ROS1 and ROS2 type strings
        const possibleTypes = ['sensor_msgs/PointCloud2', 'sensor_msgs/msg/PointCloud2'];
        for (const t of possibleTypes) {
            try {
                lidarTopic = new ROSLIB.Topic({
                    ros,
                    name: '/lidar_points',
                    messageType: t
                });
                // Only set one listener
                lidarTopic.subscribe(onLidarMsg);
                console.log("Subscribing to /lidar_points as", t);
                break;
            } catch (e) {
                console.warn("Unable to subscribe with messageType", t, e);
            }
        }
        if (!lidarTopic) {
            console.error("Failed to create lidar topic subscriber.");
            lidarStatus.textContent = "(topic error)";
        }
    }

    function onLidarMsg(msg) {
        try {
            const pts = decodePointCloud2(msg, 50000); // cap to 50k points for browser
            lastLidarPoints = pts.length;
            drawTopDown(pts);
            lidarStatus.textContent = `(ok: ${lastLidarPoints} pts)`;
        } catch (e) {
            console.error("PointCloud2 decode error:", e);
            lidarStatus.textContent = "(decode error)";
        }
    }

    // Decode PointCloud2 from rosbridge (msg.data is base64)
    function decodePointCloud2(msg, maxPoints=30000) {
        // Determine endianness
        const littleEndian = !msg.is_bigendian;
        // Decode base64 -> ArrayBuffer
        const binary = atob(msg.data);
        const len = binary.length;
        const buffer = new ArrayBuffer(len);
        const bytes = new Uint8Array(buffer);
        for (let i = 0; i < len; i++) bytes[i] = binary.charCodeAt(i);
        const view = new DataView(buffer);

        // Find offsets for x,y,z
        let offX = 0, offY = 4, offZ = 8;
        if (Array.isArray(msg.fields)) {
            for (const f of msg.fields) {
                if (f.name === 'x') offX = f.offset;
                if (f.name === 'y') offY = f.offset;
                if (f.name === 'z') offZ = f.offset;
            }
        }

        const pointStep = msg.point_step;
        // Compute number of points robustly
        let count = 0;
        if (msg.width && msg.height) {
            count = msg.width * msg.height;
        } else {
            count = Math.floor(buffer.byteLength / pointStep);
        }
        const n = Math.min(count, maxPoints);

        const pts = new Array(n);
        for (let i = 0; i < n; i++) {
            const base = i * pointStep;
            const x = view.getFloat32(base + offX, littleEndian);
            const y = view.getFloat32(base + offY, littleEndian);
            const z = view.getFloat32(base + offZ, littleEndian);
            if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
                pts[i] = [x, y, z];
            } else {
                pts[i] = [0, 0, 0];
            }
        }
        return pts;
    }

    // Simple top-down render (X forward, Y left). Z ignored for now.
    function drawTopDown(points) {
        const ctx = lidarCtx;
        const w = lidarCanvas.width;
        const h = lidarCanvas.height;
        ctx.clearRect(0, 0, w, h);

        // Draw center (vehicle)
        ctx.beginPath();
        ctx.arc(w/2, h*0.8, 4, 0, Math.PI*2);
        ctx.fillStyle = '#00ff88';
        ctx.fill();

        // Determine scale to fit ~50 m radius
        const maxRangeM = 50.0;
        const pxPerM = Math.min(w, h) / (2*maxRangeM);

        ctx.globalAlpha = 0.9;
        ctx.beginPath();
        for (let i = 0; i < points.length; i++) {
            const x = points[i][0]; // forward
            const y = points[i][1]; // left
            // Map: forward (x) -> up on screen (y negative), left (y) -> right on screen
            const sx = Math.round(w/2 + y * pxPerM);
            const sy = Math.round(h*0.8 - x * pxPerM);
            if (sx >= 0 && sx < w && sy >= 0 && sy < h) {
                ctx.rect(sx, sy, 1, 1);
            }
        }
        ctx.fillStyle = '#88c5ff';
        ctx.fill();
        ctx.globalAlpha = 1.0;

        // Range rings
        ctx.strokeStyle = '#333';
        for (let r = 10; r <= maxRangeM; r += 10) {
            ctx.beginPath();
            ctx.arc(w/2, h*0.8, r*pxPerM, 0, Math.PI*2);
            ctx.stroke();
        }
    }

    // Kick off connection
    connectRos();

    // ====================================================================
    //                  CAMERA STREAMING FUNCTIONS (WebSockets)
    // ====================================================================

    /**
    * Starts the camera stream by connecting to the dedicated WebSocket server (8081).
    */
    async function startCamera() {
        // 1. Setup UI for connection attempt
        cameraPlaceholder.style.display = 'flex';
        videoElement.style.display = 'none';
        cameraToggleStatus.textContent = 'Camera is ON (Connecting...)';
        cameraContainer.classList.add('visible');
        
        try {
            if (cameraSocket) {
                try { cameraSocket.close(); } catch (e) {}
            }

            cameraSocket = new WebSocket(CAMERA_WEBSOCKET_URL);

            cameraSocket.onopen = function() {
                isCameraOn = true;
                cameraToggleStatus.textContent = 'Camera is ON (Streaming)';
            };

            cameraSocket.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'frame' && data.frame) {
                        // 3. Render the Base64 image received from the server (YOLO annotated frame)
                        videoElement.src = 'data:image/jpeg;base64,' + data.frame;
                        
                        // Display the image element after the first frame arrives
                        cameraPlaceholder.style.display = 'none';
                        videoElement.style.display = 'block';
                    }
                } catch (e) {
                    console.error("Error parsing camera frame data:", e);
                }
            };

            cameraSocket.onclose = function() {
                stopCamera();
                cameraToggleStatus.textContent = 'Camera is OFF (Stream Ended)';
            };

            cameraSocket.onerror = function(error) {
                console.error('Camera WebSocket error. Check WSS tunnel status and URL in JS:', error);
                cameraToggleStatus.textContent = 'Camera is OFF (Error)';
                stopCamera();
            };
            

        } catch (error) {
            // This catch handles errors during the creation of the WebSocket object itself
            console.error("Error initializing camera socket: ", error); 
            cameraToggleStatus.textContent = 'Camera Unavailable (Socket Init Failed)';
            cameraToggleButton.disabled = true;
            isCameraOn = false;
        }
    }

    /**
    * Stops the camera WebSocket stream and cleans up UI state
    */
    function stopCamera() {
        try {
            if (cameraSocket && cameraSocket.readyState === WebSocket.OPEN) {
                cameraSocket.close();
            }
        } catch(e) {}
        cameraSocket = null;
        isCameraOn = false;
        videoElement.style.display = 'none';
        cameraPlaceholder.style.display = 'flex';
    }

    // Event for camera toggle button
    if (cameraToggleButton) {
        cameraToggleButton.addEventListener('click', () => {
            if (isCameraOn) {
                stopCamera();
                cameraToggleStatus.textContent = 'Camera is OFF';
                cameraToggleButton.textContent = 'Show Camera Feed';
            } else {
                startCamera();
                cameraToggleButton.textContent = 'Hide Camera Feed';
            }
        });
    }

    // ====================================================================
    //                         OBD WEBSOCKET HANDLERS
    // ====================================================================
    function setStatus(text, type='ok') {
        statusDiv.textContent = text;
        statusDiv.classList.remove('ok', 'warn', 'error');
        statusDiv.classList.add(type);
    }

    function resetDashboard() {
        for (const key in metricsEls) {
            const element = metricsEls[key];
            if (element) {
                element.textContent = '--';
                if (key === 'ignitionState') element.classList.remove('Off', 'On', 'Running');
                if (key === 'dtc') element.classList.remove('has-dtc', 'no-dtc');
            }
        }
    }

    socket.onopen = function() {
        setStatus('Status: Connected');
    };

    socket.onmessage = function(event) {
        try {
            const payload = JSON.parse(event.data);
            if (payload.type === 'obd_data') {
                const data = payload.data;

                for (const key in metricsEls) {
                    const element = metricsEls[key];
                    const value = data[key];

                    if (value === null || value === undefined || value === 'N/A') {
                        element.textContent = '--';
                        if (key === 'ignitionState') element.classList.remove('Off', 'On', 'Running');
                        if (key === 'dtc') element.classList.remove('has-dtc', 'no-dtc');
                        continue;
                    }
                    
                    if (key === 'ignitionState') {
                        element.textContent = value;
                        element.classList.remove('Off', 'On', 'Running');
                        element.classList.add(value);
                    } else if (key === 'dtc') {
                        const hasDTCs = value && value !== 'None'; 
                        element.textContent = hasDTCs ? value : 'None';
                        element.classList.remove('has-dtc', 'no-dtc');
                        element.classList.add(hasDTCs ? 'has-dtc' : 'no-dtc');
                    } else if (key === 'idlingTime') {
                        const date = new Date(0);
                        date.setSeconds(value);
                        element.textContent = date.toISOString().substr(11, 8);
                    } else if (typeof value === 'number') {
                        element.textContent = Number(value).toFixed(1);
                    } else {
                        element.textContent = value;
                    }
                }
            }
        } catch (e) {
            console.error('Invalid message from OBD socket:', e);
        }
    };

    socket.onclose = function(event) {
        setStatus('Status: Disconnected from OBD server. Please restart the Python server and refresh.', 'error');
        resetDashboard();
        stopCamera(); 
    };

    socket.onerror = function(error) {
        setStatus('Status: OBD connection error. Is the Python server running?', 'error');
        resetDashboard();
        stopCamera(); 
    };

}); // <--- END OF DOMContentLoaded WRAPPER
