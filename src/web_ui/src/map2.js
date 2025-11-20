class Robot {
    constructor(ip, conv_circle, conv_line, conv_image, conv_toribe, x, y, theta, radius) {
        this.ip = ip;
        this.conv_circle = conv_circle;
        this.conv_line = conv_line;
        this.conv_image = conv_image;
        this.conv_toribe = conv_toribe;
        this.joint_circle = null;
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.radius = radius;
        this.routes = [];
        this.routes_line = [];

        // Add trailer physics properties
        this.toribe_x = x;
        this.toribe_y = y;
        this.toribe_theta = theta;
        this.prev_x = x;
        this.prev_y = y;
        this.prev_theta = theta;

        this.color_r = 0;
        this.color_g = 0;
        this.color_b = 0;

        this.has_finished_routes_init = false;
        this.prev_has_finished_routes_init = false;
        this.has_route_drawed = false;
    }
}

let robots = [];
let wtf_skala = 10;
// let wtf_skala = 60;
var ip_server = window.location.hostname; // IP towing robot
// var ip_server = "127.0.0.1"; // IP towing robot
// var ip_server = "192.168.24.59"; // IP towing robot
let last_time_connect = new Date();

// Create a Konva Stage
const stage = new Konva.Stage({
    container: 'map',
    width: window.innerWidth,
    height: window.innerHeight,
    // position: {x: 776.1564000000002, y: 132.96359999999993},
    // scale: {x: 0.5499159600000001, y: 0.5499159600000001}
    position: { x: 909.0812167763638, y: 233.76107358727256 },
    scale: { x: 0.8051319570360004, y: 0.8051319570360004 }
});


// Create a layer for grid and shapes
const robotLayer = new Konva.Layer();
// const mapLayer = new Konva.Layer();
const waypointsLayer = new Konva.Layer();
// stage.add(mapLayer);
stage.add(waypointsLayer);
stage.add(robotLayer);

// ================================================================================================================================

// Enable zooming
stage.on('wheel', (e) => {
    e.evt.preventDefault();
    const scaleBy = 1.1;
    const oldScale = stage.scaleX();
    const pointer = stage.getPointerPosition();
    const mousePointTo = {
        x: (pointer.x - stage.x()) / oldScale,
        y: (pointer.y - stage.y()) / oldScale,
    };

    const newScale = e.evt.deltaY > 0 ? oldScale / scaleBy : oldScale * scaleBy;
    stage.scale({ x: newScale, y: newScale });

    const newPos = {
        x: pointer.x - mousePointTo.x * newScale,
        y: pointer.y - mousePointTo.y * newScale,
    };
    stage.position(newPos);
    stage.batchDraw();
});

// Enable map shifting (panning) with the right mouse button
let isDragging = false;
let dragStartPos = { x: 0, y: 0 };

stage.on('mousedown', (e) => {
    if (e.evt.button === 2) { // Check if the right mouse button is pressed
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }

    if (e.evt.button === 1 || e.evt.button === 2) {
        isDragging = true;
        dragStartPos = stage.getPointerPosition();
    }
});

stage.on('mousemove', (e) => {
    if (!isDragging) return;

    const pointer = stage.getPointerPosition();
    const dx = pointer.x - dragStartPos.x;
    const dy = pointer.y - dragStartPos.y;

    stage.position({
        x: stage.x() + dx,
        y: stage.y() + dy,
    });
    stage.batchDraw();
    dragStartPos = pointer;
});

stage.on('mouseup', () => {
    isDragging = false;
});

stage.on('contextmenu', (e) => {
    // Prevent the browser's context menu from appearing on right-click
    e.evt.preventDefault();
});

// Handle window resizing
window.addEventListener('resize', () => {
    const width = window.innerWidth;
    const height = window.innerHeight;

    stage.width(width);
    stage.height(height);

    robotLayer.batchDraw();
    // mapLayer.batchDraw();
    // waypointsLayer.batchDraw();
});

// ================================================================================================================================
function normalizeAngle(angle) {
    while (angle > Math.PI) {
        angle -= 2 * Math.PI;
    }
    while (angle < -Math.PI) {
        angle += 2 * Math.PI;
    }
    return angle;
}

function addRobotImageWithToribe(ip, x, y, theta, radius, imageScale) {
    // console.log(`addRobotImageWithToribe called: ip=${ip}, x=${x}, y=${y}, theta=${(theta * 180/Math.PI).toFixed(1)}°`);

    const original_x = x;
    const original_y = y;
    const original_theta = theta;

    // Transform coordinates
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;

    // Check if robot already exists and UPDATE it
    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            // Update existing robot positions with realistic trailer physics
            const towingX = x * wtf_skala;
            const towingY = y * wtf_skala;

            // Calculate velocity and steering angle
            const dx = original_x - robots[i].prev_x;
            const dy = original_y - robots[i].prev_y;
            const velocity = Math.sqrt(dx * dx + dy * dy);
            const dt = 1.5; // 50ms time step

            // Trailer physics parameters
            const L = radius * 2.5; // Distance between axles (hitch length)
            const trailer_L = radius * 2.5; // Trailer wheelbase

            // Calculate steering angle from velocity with angle normalization
            const dtheta = normalizeAngle(original_theta - robots[i].prev_theta);
            let steering_angle = 0;
            if (velocity > 0.01) {
                steering_angle = Math.atan2(dtheta * L, velocity * dt);
                // Limit steering angle
                steering_angle = Math.max(-Math.PI / 4, Math.min(Math.PI / 4, steering_angle));
            }

            // Update trailer position using bicycle model
            if (velocity > 0.01) {
                // Calculate target trailer angle with proper normalization
                const trailer_theta_target = normalizeAngle(original_theta - steering_angle);
                const current_trailer_theta = normalizeAngle(robots[i].toribe_theta);

                // Smooth trailer rotation with proper angle difference handling
                const angle_diff = normalizeAngle(trailer_theta_target - current_trailer_theta);
                robots[i].toribe_theta = normalizeAngle(current_trailer_theta + angle_diff * 0.3);

                // Calculate trailer position based on physics
                const joint_offset_x = -L * Math.cos(original_theta);
                const joint_offset_y = -L * Math.sin(original_theta);
                const trailer_offset_x = -trailer_L * Math.cos(robots[i].toribe_theta);
                const trailer_offset_y = -trailer_L * Math.sin(original_theta);

                robots[i].toribe_x = original_x + (joint_offset_x + trailer_offset_x) / wtf_skala;
                robots[i].toribe_y = original_y + (joint_offset_y + trailer_offset_y) / wtf_skala;
            }

            // Update towing vehicle position and rotation
            robots[i].conv_image.position({
                x: towingX,
                y: towingY
            });
            robots[i].conv_image.rotation(90 + original_theta * -180 / Math.PI);

            // Calculate joint position (hitch point)
            const jointDistance = radius * 1.2 * wtf_skala;
            const jointX = towingX - jointDistance * Math.cos(original_theta);
            const jointY = towingY + jointDistance * Math.sin(original_theta);

            // Update joint position
            if (robots[i].joint_circle) {
                robots[i].joint_circle.position({
                    x: jointX,
                    y: jointY
                });
            }

            const toribeDistance = radius * 1.8 * wtf_skala; // Jarak toribe dari joint
            const toribeX = jointX - toribeDistance * Math.cos(robots[i].toribe_theta); // Toribe di belakang joint
            const toribeY = jointY + toribeDistance * Math.sin(robots[i].toribe_theta); // Toribe di belakang joint

            // Update toribe position BERDASARKAN JOINT, bukan screen coordinates
            if (robots[i].conv_toribe) {
                robots[i].conv_toribe.position({
                    x: toribeX,
                    y: toribeY
                });
                robots[i].conv_toribe.rotation(90 + robots[i].toribe_theta * -180 / Math.PI);
            }
            // Update direction line
            if (robots[i].conv_line) {
                robots[i].conv_line.points([
                    towingX, towingY,
                    towingX + radius * wtf_skala * Math.cos(original_theta),
                    towingY - radius * wtf_skala * Math.sin(original_theta)
                ]);
            }

            // Store previous values for next iteration
            robots[i].prev_x = original_x;
            robots[i].prev_y = original_y;
            robots[i].prev_theta = original_theta;
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;

            robotLayer.batchDraw();
            return; // IMPORTANT: Return here to prevent creating new robot
        }
    }

    // Only create new robot if it doesn't exist
    // console.log(`Creating NEW robot with IP: ${ip}`);

    // Check if images are loaded
    if (!towingimage.complete || !toribeimage.complete) {
        console.error("Images not loaded yet");
        return;
    }

    // IMPORTANT: Remove any existing robot with same IP before creating new one
    robots = robots.filter(robot => {
        if (robot.ip === ip) {
            // console.log(`Removing existing robot with IP: ${ip}`);
            if (robot.conv_image) robot.conv_image.destroy();
            if (robot.conv_toribe) robot.conv_toribe.destroy();
            if (robot.conv_line) robot.conv_line.destroy();
            if (robot.joint_circle) robot.joint_circle.destroy();
            if (robot.conv_circle) robot.conv_circle.destroy();
            return false; // Remove this robot
        }
        return true; // Keep other robots
    });

    const towingX = x * wtf_skala;
    const towingY = y * wtf_skala;

    // Create towing vehicle image
    const conv_image = new Konva.Image({
        x: towingX,
        y: towingY,
        image: towingimage,
        width: radius * 2 * wtf_skala,
        height: radius * 2 * wtf_skala,
        offsetX: radius * wtf_skala,
        offsetY: radius * wtf_skala,
        rotation: 90 + original_theta * -180 / Math.PI,
        scale: { x: imageScale, y: imageScale },
        draggable: false,
    });

    // Calculate joint position (hitch point)
    const jointDistance = radius * 1.2 * wtf_skala;
    const jointX = towingX - jointDistance * Math.cos(original_theta);
    const jointY = towingY + jointDistance * Math.sin(original_theta);

    // Create joint circle (red connection point)
    const joint_circle = new Konva.Circle({
        x: jointX,
        y: jointY,
        radius: radius * 0.3 * wtf_skala,
        fill: 'red',
        stroke: 'darkred',
        strokeWidth: 2,
    });

    // Initial trailer position (behind towing vehicle)
    const toribeDistance = radius * 1.8 * wtf_skala;
    const toribeX = jointX - toribeDistance * Math.cos(original_theta); // Berdasarkan joint
    const toribeY = jointY + toribeDistance * Math.sin(original_theta); // Berdasarkan joint

    // Create toribe image BERDASARKAN JOINT POSITION
    const conv_toribe = new Konva.Image({
        x: toribeX,      // PERBAIKAN: Gunakan toribeX yang dihitung dari joint
        y: toribeY,      // PERBAIKAN: Gunakan toribeY yang dihitung dari joint
        image: toribeimage,
        width: radius * 2 * wtf_skala,
        height: radius * 2 * wtf_skala,
        offsetX: radius * wtf_skala,
        offsetY: radius * wtf_skala,
        rotation: 90 + original_theta * -180 / Math.PI,
        scale: { x: imageScale * 1.3, y: imageScale * 1.3 },
        draggable: false,
    });

    // Create direction line for towing vehicle
    const conv_line = new Konva.Line({
        points: [
            towingX, towingY,
            towingX + radius * wtf_skala * Math.cos(original_theta),
            towingY - radius * wtf_skala * Math.sin(original_theta)
        ],
        stroke: 'cyan',
        strokeWidth: 5,
    });

    // Create robot object with physics tracking
    let robot_buffer = new Robot(ip, null, conv_line, conv_image, conv_toribe, original_x, original_y, original_theta, radius);
    robot_buffer.joint_circle = joint_circle;

    // Initialize trailer physics with normalized angles
    robot_buffer.toribe_x = (toribeX / wtf_skala) - stage.width() * 0.5 / wtf_skala; // Convert back to world coords
    robot_buffer.toribe_y = stage.height() * 0.5 / wtf_skala - (toribeY / wtf_skala); // Convert back to world coords
    robot_buffer.toribe_theta = normalizeAngle(original_theta);
    robot_buffer.prev_x = original_x;
    robot_buffer.prev_y = original_y;
    robot_buffer.prev_theta = normalizeAngle(original_theta);

    robots.push(robot_buffer);

    // Add all elements to layer (order matters for layering)
    robotLayer.add(conv_toribe);     // Toribe at bottom
    // robotLayer.add(joint_circle);    // Joint in middle
    robotLayer.add(conv_image);      // Towing vehicle on top
    robotLayer.add(conv_line);       // Direction line on top

    robotLayer.draw();

}

function addRobotImage(ip, x, y, theta, radius, imageScale) {
    const original_x = x;
    const original_y = y;
    const original_theta = theta;

    // Transform coordinates
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;

    // Check if robot already exists
    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            // Update existing robot image position and rotation
            robots[i].conv_image.position({
                x: x * wtf_skala,
                y: y * wtf_skala
            });
            robots[i].conv_image.rotation(90 + theta * -180 / Math.PI); // Convert radians to degrees
            robots[i].conv_line.points([
                x * wtf_skala, y * wtf_skala,
                x * wtf_skala + radius * wtf_skala * Math.cos(theta),
                y * wtf_skala - radius * wtf_skala * Math.sin(theta)
            ]);
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;

            robotLayer.batchDraw();
            return;
        }
    }

    const conv_image = new Konva.Image({
        x: x * wtf_skala,
        y: y * wtf_skala,
        image: towingimage,
        width: radius * 2 * wtf_skala,
        height: radius * 2 * wtf_skala,
        offsetX: radius * wtf_skala,
        offsetY: radius * wtf_skala,
        rotation: theta * 180 / Math.PI, // Convert radians to degrees for Konva
        scale: { x: imageScale, y: imageScale },
        draggable: false,
    });

    // Calculate the end point of the line based on theta
    const thetaRadians = theta; // Convert to radians
    const lineEndX = x * wtf_skala + radius * wtf_skala * Math.cos(thetaRadians);
    const lineEndY = y * wtf_skala - radius * wtf_skala * Math.sin(thetaRadians);

    // Draw the line inside the circle
    const conv_line = new Konva.Line({
        points: [x * wtf_skala, y * wtf_skala, lineEndX, lineEndY],
        stroke: 'Cyan',
        strokeWidth: 5,
    });


    // Create robot object and add to array
    let robot_buffer = new Robot(ip, null, conv_line, conv_image, x, y, theta, radius);
    robots.push(robot_buffer);

    robotLayer.add(robot_buffer.conv_image);
    robotLayer.add(robot_buffer.conv_line);
    robotLayer.draw();
}

function addRobot(ip, x, y, theta, radius, colourr) {
    const original_x = x;
    const original_y = y;
    const original_theta = theta;

    ip = ip;
    x = x + stage.width() * 0.5 / wtf_skala;
    y = stage.height() * 0.5 / wtf_skala - y;
    theta = theta;
    radius = radius;


    for (let i = 0; i < robots.length; i++) {
        if (robots[i].ip == ip) {
            robots[i].conv_circle.position({ x: x * wtf_skala, y: y * wtf_skala });
            robots[i].conv_line.points([x * wtf_skala, y * wtf_skala, x * wtf_skala + radius * wtf_skala * Math.cos(theta), y * wtf_skala - radius * wtf_skala * Math.sin(theta)]);
            robots[i].x = original_x;
            robots[i].y = original_y;
            robots[i].theta = original_theta;
            robots[i].radius = radius;

            robotLayer.batchDraw();

            return;
        }
    }

    const conv_circle = new Konva.Circle({
        x: x * wtf_skala,
        y: y * wtf_skala,
        radius: radius * wtf_skala,
        fill: colourr,
        draggable: true,
    });

    // Calculate the end point of the line based on theta
    const thetaRadians = theta; // Convert to radians
    const lineEndX = x * wtf_skala + radius * wtf_skala * Math.cos(thetaRadians);
    const lineEndY = y * wtf_skala - radius * wtf_skala * Math.sin(thetaRadians);

    // Draw the line inside the circle
    const conv_line = new Konva.Line({
        points: [x * wtf_skala, y * wtf_skala, lineEndX, lineEndY],
        stroke: 'Cyan',
        strokeWidth: 5,
    });

    let robot_buffer = new Robot(ip, conv_circle, conv_line, x, y, theta, radius);

    robots.push(robot_buffer);

    robotLayer.add(robot_buffer.conv_circle);
    robotLayer.add(robot_buffer.conv_line);
    robotLayer.draw();
}

// ================================================================================================================================

// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

// Battery Animation Functions
function updateBatteryAnimation(socValue, isCharging = false) {
    const batteryLevel = document.getElementById('battery-level');
    const batteryPercentage = document.getElementById('battery-percentage');
    const batteryStatus = document.getElementById('battery-status');
    const batteryIcon = document.getElementById('battery-icon');
    const chargingBolt = document.getElementById('charging-bolt');

    // Update percentage text
    batteryPercentage.textContent = `${socValue}%`;

    // Update battery level HEIGHT (untuk vertikal)
    batteryLevel.style.height = `${socValue}%`;

    // Remove all battery classes
    batteryLevel.classList.remove('battery-high', 'battery-medium', 'battery-low', 'battery-critical');
    batteryIcon.classList.remove('battery-charging');

    // Add appropriate color class based on SOC
    if (socValue >= 75) {
        batteryLevel.classList.add('battery-high');
        batteryStatus.textContent = 'Excellent';
    } else if (socValue > 40) {
        batteryLevel.classList.add('battery-high');
        batteryStatus.textContent = 'Good';
    } else if (socValue <= 40 && socValue > 30) {
        batteryLevel.classList.add('battery-medium');
        batteryStatus.textContent = 'Low';
    } else if (socValue <= 30 && socValue > 0) {
        batteryLevel.classList.add('battery-low');
        batteryStatus.textContent = 'Critical';
    }

    // Handle charging animation
    if (isCharging) {
        batteryIcon.classList.add('battery-charging');
        chargingBolt.style.display = 'block';
        batteryStatus.textContent = 'Charging';
    } else {
        chargingBolt.style.display = 'none';
    }

    // Animate battery level change (HEIGHT untuk vertikal)
    anime({
        targets: batteryLevel,
        height: `${socValue}%`,
        duration: 1000,
        easing: 'easeOutQuart'
    });

    // Animate percentage number
    anime({
        targets: { value: parseInt(batteryPercentage.textContent) || 0 },
        value: socValue,
        duration: 1000,
        easing: 'easeOutQuart',
        update: function (anim) {
            batteryPercentage.textContent = `${Math.round(anim.animatables[0].target.value)}%`;
        }
    });
}

// Add disconnection handler to reset battery
function resetBatteryAnimation() {
    const batteryStatus = document.getElementById('battery-status');
    batteryStatus.textContent = 'Disconnected';
    updateBatteryAnimation(0, false);
}
// ...existing code...

function updateWiFiWidget(latency) {
    const wifiStatus = document.getElementById('wifi-status');
    const wifiContainer = document.getElementById('wifi-container');
    let signalStrength = 0;
    let statusClass = '';

    // Calculate signal strength based on latency
    if (latency < 50) {
        signalStrength = 4; // Excellent
        statusClass = 'wifi-excellent';
        wifiStatus.textContent = `${latency}ms`;
        wifiStatus.style.color = 'white';
    } else if (latency < 100) {
        signalStrength = 3; // Good
        statusClass = 'wifi-good';
        wifiStatus.textContent = `${latency}ms`;
        wifiStatus.style.color = 'white';
    } else if (latency < 200) {
        signalStrength = 2; // Fair
        statusClass = 'wifi-fair';
        wifiStatus.textContent = `${latency}ms`;
        wifiStatus.style.color = 'white';
    } else if (latency < 500) {
        signalStrength = 1; // Poor
        statusClass = 'wifi-poor';
        wifiStatus.textContent = `${latency}ms`;
        wifiStatus.style.color = 'white';
    } else {
        signalStrength = 0; // No signal
        statusClass = 'wifi-no-signal';
        wifiStatus.textContent = 'No Signal';
        wifiStatus.style.color = 'red';
    }

    // Remove all WiFi classes and add new one
    wifiContainer.className = 'wifi-container ' + statusClass;

    // Update individual bars
    for (let i = 1; i <= 4; i++) {
        const bar = document.getElementById(`wifi-bar-${i}`);
        if (bar) {
            if (i <= signalStrength) {
                bar.style.opacity = '1';
            } else {
                bar.style.opacity = '0.3';
            }
        }
    }

}
// ...existing code...

let last_time_update_map = 0;
let mapCanvas = document.createElement("canvas");
let mapCtx = mapCanvas.getContext("2d");

const EMERGENCY_LIDAR_DEPAN_DETECTED = 0b010
const EMERGENCY_CAMERA_OBS_DETECTED = 0b100
const EMERGENCY_GYRO_ANOMALY_DETECTED = 0b1000
const EMERGENCY_ICP_SCORE_TERLALU_BESAR = 0b10000
const EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR = 0b100000
const EMERGENCY_STOP_KARENA_OBSTACLE = 0b1000000
const EMERGENCY_GANDENGAN_LEPAS = 0b100000000
const STATUS_TOWING_ACTIVE_AUTO = 0b01

const status_emergency = document.getElementById('status-emergency');
const terminal_terakhir = document.getElementById('terminal-terakhir');
const counter_lap = document.getElementById('counter-lap');
const label_lap = document.getElementById('label-lap');
const cam_main = document.getElementById('cam-main');

let isTryingToConnect = false;

const alarm = new Audio('alarm1.wav');
alarm.loop = true;
let test_audio_play = 0

const alarm_40 = new Audio('alarm40.wav');
alarm_40.loop = true;

const alarm_30 = new Audio('alarm30.wav');
alarm_30.loop = true;

const towingimage = new Image();
towingimage.src = 'towing.png';

const toribeimage = new Image();
toribeimage.src = 'toribe.png';
// ================================================================

let wp_subscriber = null;
let t2_odometry_subscriber = null;
let t2_map_subscriber = null;
let t2_status_emergency_subscriber = null;
let t2_terminal_terakhir_subscriber = null;
let t2_soc = null;
let t2_counter_lap = null;
let t2_lag_ms = null;
let t2_lap_sum = null;

let is_lap_subscribed = false;

let currentStatus = null;
let lastLoggedStatus = null;
let lastTerminalStatus = null;
let lastRosStatus = null;

let currentSOC = 0;
let currentLap = 0;
let currentX = 0.0;
let currentY = 0.0;
let currentTheta = 0.0;


function getCurrentDateString() {
    const now = new Date();
    const year = now.getFullYear();
    const month = String(now.getMonth() + 1).padStart(2, '0');
    const day = String(now.getDate()).padStart(2, '0');
    return `${year}-${month}-${day}`;
}

function getCurrentTimestamp() {
    const now = new Date();
    return now.toISOString();
}

function logCurrentRobotState() {
    if (!is_lap_subscribed) {
        return;
    }
    if (!lastTerminalStatus) {
        return;
    }

    const logData = {
        timestamp: new Date().toISOString(),
        dateString: getCurrentDateString(),
        terminal: lastTerminalStatus,
        warning: currentStatus || "Towing Normal",
        soc: currentSOC,
        lap: currentLap,
        x: currentX,
        y: currentY,
        theta: currentTheta
    };

    // fetch('http://${ip_server}:3001/log-robot-data', {
    //     method: 'POST',
    //     headers: {
    //         'Content-Type': 'application/json',
    //     },
    //     body: JSON.stringify(logData)
    // })
    //     .then(response => response.json());

    fetch('http://' + ip_server + ':3002/log-robot-data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(logData)
    })
        .then(response => response.json());
}

// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://" + ip_server + ":9090",
});

function connectRos() {
    if (isTryingToConnect) return;

    isTryingToConnect = true;
    console.log('Attempting to connect to rosbridge...');

    ros.connect('ws://' + ip_server + ':9090');
}

function updateDateTimeDisplay() {
    const dateElement = document.getElementById('date-display');
    const timeElement = document.getElementById('time-display');
    if (!dateElement || !timeElement) return;

    const now = new Date();

    const dateOptions = {
        weekday: 'long',
        year: 'numeric',
        month: 'long',
        day: 'numeric'
    };
    const timeOptions = {
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
        hour12: true
    };

    dateElement.textContent = now.toLocaleDateString('id-ID', dateOptions);
    timeElement.textContent = now.toLocaleTimeString('en-US', timeOptions);
}

updateDateTimeDisplay();
setInterval(updateDateTimeDisplay, 1000);
// First connection attempt
connectRos();

// Try to reconnect every 3 seconds if not connected
let lastReconnectAttempt = 0;
setInterval(() => {
    // console.log(stage);
    let time_now = new Date();
    let time_diff = time_now - last_time_connect;
    if (Math.floor(time_diff / 1000) > 30) {
        isTryingToConnect = false;
        //	connectRos();
        location.reload();
    }
    console.log("Time: ", time_diff);


    if (!ros.isConnected) {
        status_emergency.innerHTML = "Towing Disconnected";

        // Log reconnection attempt only once every 30 seconds to avoid spam
        const now = Date.now();
        if (now - lastReconnectAttempt > 50000) {
            lastReconnectAttempt = now;
        }

        connectRos();
    }
}, 3000);


function reconnectImg(elemOrId, baseUrl) {
    const oldImg = typeof elemOrId === 'string' ? document.getElementById(elemOrId) : elemOrId;

    // Buat IMG baru dari nol (tanpa koneksi/event lama)
    const fresh = new Image();
    fresh.width = oldImg.width;
    fresh.height = oldImg.height;
    fresh.alt = oldImg.alt || '';

    // (Opsional) copy class/style
    fresh.className = oldImg.className;
    fresh.style.cssText = oldImg.style.cssText;

    // Pasang handler sebelum set src
    fresh.onload = () => {
        // sukses → lepas dan buang img lama
        oldImg.replaceWith(fresh);
    };
    fresh.onerror = () => {
        // gagal → coba lagi dengan backoff kecil
        setTimeout(() => reconnectImg(fresh, baseUrl), 1000);
    };

    // Putus koneksi lama sebersih mungkin
    oldImg.src = '';
    oldImg.removeAttribute('src');      // beberapa browser perlu ini

    // Paksa URL unik (cache-buster)
    const url = baseUrl + (baseUrl.includes('?') ? '&' : '?') + 'ts=' + Date.now();
    fresh.src = url;
}

cam_main.src = 'http://' + ip_server + ':7890/cam1.mjpeg';
let last_time_camera_normal = 0;
setInterval(() => {
    let time_now = Date.now();
    let time_diff = time_now - last_time_camera_normal;
    console.log("Camera time diff: ", Math.floor(time_diff / 1000));
    if (Math.floor(time_diff / 1000) > 10) {
        console.log("Camera seems disconnected, refreshing...");
        reconnectImg(cam_main, 'http://' + ip_server + ':7890/cam1.mjpeg');
    }

}, 3000);

cam_main.onload = function () {
    last_time_camera_normal = Date.now();
    console.log('Camera image loaded successfully.');
};


async function loadLapSum() {
    try {
        const res = await fetch('http://' + ip_server + ':3002/lap-sum', {
            method: 'GET',
            mode: 'cors',
            headers: { 'Accept': 'application/json' }
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);

        // The endpoint returns a JSON number (e.g., 123)
        let total = await res.json();

        // Fallback if server ever returns a string (e.g., "123")
        if (typeof total !== 'number') {
            total = Number.parseInt(total, 10);
        }

        console.log('Lap Sum:', total);

        const formattedLap = total.toString().padStart(2, '0');
        counter_lap.innerHTML = formattedLap;
        label_lap.style.color = 'white';
        counter_lap.style.color = 'white'

    } catch (err) {
        console.error('Failed to fetch lap sum:', err);
    }
}

// setInterval(() => {
//     loadLapSum();
// }, 2000)

// CAPEKKKKK, NEXT menambahkan terminal terakhir
ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
    is_lap_subscribed = false;
    const connectionStatus = "ROS Connected";

    // ===================================================
    t2_counter_lap = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t2/counter_lap',
        messageType: 'std_msgs/Int32'
    });
    t2_counter_lap.subscribe(function (message) {
        const lap = message.data;
        currentLap = lap;
        is_lap_subscribed = true;
    });

    t2_lag_ms = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t2/lag_ms',
        messageType: 'std_msgs/Int16'
    });
    t2_lag_ms.subscribe(function (message) {
        updateWiFiWidget(message.data)
    });

    t2_lap_sum = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t2/lap_sum',
        messageType: 'std_msgs/Int16'
    });
    t2_lap_sum.subscribe(function (message) {
        const formattedLap = message.data.toString().padStart(2, '0');
        counter_lap.innerHTML = formattedLap;
        label_lap.style.color = 'white';
        counter_lap.style.color = 'white'
    });


    t2_soc = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t2/battery_soc',
        messageType: 'std_msgs/Int16'
    });

    t2_soc.subscribe(function (message) {
        const soc = message.data;
        currentSOC = soc;

        updateBatteryAnimation(soc, false); // Set to true if you want charging animation

        // Change color based on SOC value
        if (soc < 5) {
            if (test_audio_play == 0) {
                if (!alarm_30.paused) {
                    alarm_30.pause();
                    alarm_30.currentTime = 0;
                }
                if (!alarm_40.paused) {
                    alarm_40.pause();
                    alarm_40.currentTime = 0;
                }
            }
        }
        else if (soc < 30) {
            if (alarm_30.paused) {
                alarm_30.play();
            }
        } else if (soc < 40) {
            if (alarm_30.paused) {
                alarm_30.play();
            }
        } else {
            if (test_audio_play == 0) {
                if (!alarm_30.paused) {
                    alarm_30.pause();
                    alarm_30.currentTime = 0;
                }
                if (!alarm_40.paused) {
                    alarm_40.pause();
                    alarm_40.currentTime = 0;
                }
            }
        }
    });

    wp_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/waypoints',
        messageType: 'sensor_msgs/PointCloud'
    });

    wp_subscriber.subscribe(function (message) {
        const points = message.points;
        const waypoints = [];

        for (let i = 0; i < points.length; i++) {
            const x = points[i].x;
            const y = points[i].y;

            const x_tf = x + stage.width() * 0.5 / wtf_skala;
            const y_tf = stage.height() * 0.5 / wtf_skala - y;

            waypoints.push(x_tf * wtf_skala, y_tf * wtf_skala);
        }

        // Clear the shape layer
        waypointsLayer.destroyChildren();

        // Draw the waypoints
        const waypointsLine = new Konva.Line({
            points: waypoints,
            stroke: 'red',
            strokeWidth: 15,
        });
        waypointsLayer.add(waypointsLine);

        waypointsLayer.draw();

        wp_subscriber.unsubscribe();
    }
    );

    t2_odometry_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t2/pose_filtered',
        messageType: 'nav_msgs/Odometry'
    });

    t2_odometry_subscriber.subscribe(function (message) {
        last_time_connect = new Date();
        lastMessageTime = Date.now(); // Track last message
        const x = message.pose.pose.position.x;
        const y = message.pose.pose.position.y;
        const q = message.pose.pose.orientation;
        const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const radius = 4.05;

        currentX = x;
        currentY = y;
        currentTheta = theta;

        // addRobot("0.0.0.0", x, y, theta, radius, 'red');
        // addRobotImage("0.0.0.0", x, y, theta, radius, 1.5);
        addRobotImageWithToribe("0.0.0.0", x, y, theta, radius, 2.0);

        // logCurrentRobotState();
    });

    t2_status_emergency_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: "/udp/t2/status_emergency",
        messageType: "std_msgs/Int16",
    });

    t2_status_emergency_subscriber.subscribe(function (message) {
        let newStatus = null;
        const rawData = message.data;

        if ((message.data & STATUS_TOWING_ACTIVE_AUTO) == 0) {
            newStatus = "Towing Mode Manual";
            status_emergency.innerHTML = newStatus;
            if (!alarm.paused) {
                alarm.pause();
                alarm.currentTime = 0;
            }
        }
        else if ((message.data & EMERGENCY_STOP_KARENA_OBSTACLE) == EMERGENCY_STOP_KARENA_OBSTACLE) {
            newStatus = "WARNING: Towing Berhenti Karena Obstacle";
            status_emergency.innerHTML = newStatus;
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_GYRO_ANOMALY_DETECTED) == EMERGENCY_GYRO_ANOMALY_DETECTED) {
            newStatus = "WARNING: Gyro Anomali Terdeteksi";
            status_emergency.innerHTML = newStatus;
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_ICP_SCORE_TERLALU_BESAR) == EMERGENCY_ICP_SCORE_TERLALU_BESAR) {
            newStatus = "WARNING: Anomali Lingkungan Terdeteksi";
            status_emergency.innerHTML = newStatus;
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR) == EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR) {
            newStatus = "WARNING: Hipotesis Kesalahan Posisi";
            status_emergency.innerHTML = newStatus;
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_GANDENGAN_LEPAS) == EMERGENCY_GANDENGAN_LEPAS) {
            status_emergency.innerHTML = "WARNING: TIDAK ADA TORIBE";
            if (alarm.paused) {
                alarm.play();
            }
        }
        else if ((message.data & EMERGENCY_LIDAR_DEPAN_DETECTED) == EMERGENCY_LIDAR_DEPAN_DETECTED) {
            newStatus = "WARNING: Lidar Mendeteksi Objek";
            status_emergency.innerHTML = newStatus;
        }
        else if ((message.data & EMERGENCY_CAMERA_OBS_DETECTED) == EMERGENCY_CAMERA_OBS_DETECTED) {
            newStatus = "WARNING: Kamera Mendeteksi Objek";
            status_emergency.innerHTML = newStatus;
        }
        else {
            newStatus = "Towing Normal";
            status_emergency.innerHTML = newStatus;
            if (test_audio_play == 0) {
                if (!alarm.paused) {
                    alarm.pause();
                    alarm.currentTime = 0;
                }
            }
        }
        const oldStatus = currentStatus;
        // Selalu update status saat ini
        currentStatus = newStatus;

        const isImportantWarning = newStatus !== "Towing Normal";

        // if (newStatus !== oldStatus && isImportantWarning) {
        //     logCurrentRobotState();
        // }
    });

    // ===================================================

    t2_terminal_terakhir_subscriber = new ROSLIB.Topic({
        ros: ros,
        name: "/udp/t2/terminal_terakhir",
        messageType: "std_msgs/Int16",
    });

    t2_terminal_terakhir_subscriber.subscribe(function (message) {
        let terminal_terakhir_value = message.data;
        let terminalStatus = null;

        //0 Berangkat: Area jibcrane IST 
        // 26 Berangkat: Tikungan samping lab 
        // 24 Berangkat: Jalur 1 lurus depan YOKAI 1
        // 1 Berangkat: Tikungan samping yokai 1
        // 3 Berangkat: Jalur 1 lurus samping yokai 1 
        // 5/6 Berangkat: Jalur 1 Tikungan Bawah tangga 
        // 11 Berangkat: Jalur 1 lurus setelah bawah tangga 
        // 15 Berangkat: Jalur 1 tikungan beacukai 
        // 19 Berangkat: Jalur 1 lurus setelah beacukai 
        // 23/46 Berangkat: Degasing 
        // 7 Pulang: Degasing 
        // 25/47 Pulang: Degasing 
        // 35 Pulang: Jalur 2 lurus setelah beacukai
        // 37 Pulang: Jalur 2 Tikungan beacukai 
        // 38 Pulang: Jalur 2 lurus sebelum beacukai 
        // 40 Pulang: Jalur 2 tikungan bawah tangga 
        // 41 Pulang: Jalur 2 lurus samping yokai 1 
        // 43/48 Pulang: Jalur 2 Tikungan jembatan penyebrangan
        if (terminal_terakhir_value == -1) {
            terminalStatus = "Terminal Terakhir: Tempat Parkir";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 0) {
            terminalStatus = "Terminal Terakhir: Area Jibcrane IST (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 1) {
            terminalStatus = "Terminal Terakhir: Tikungan Samping Yokai 1 (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 3) {
            terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Samping Yokai 1 (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 5 || terminal_terakhir_value == 6) {
            terminalStatus = "Terminal Terakhir: Jalur 1 Tikungan Bawah Tangga (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 7) {
            terminalStatus = "Terminal Terakhir: Degasing (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 11) {
            terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Setelah Bawah Tangga (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 15) {
            terminalStatus = "Terminal Terakhir: Jalur 1 Tikungan Beacukai (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 19) {
            terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Setelah Beacukai (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 23 || terminal_terakhir_value == 46) {
            terminalStatus = "Terminal Terakhir: Degasing (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 24) {
            terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Depan Yokai 1 (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 25 || terminal_terakhir_value == 47) {
            terminalStatus = "Terminal Terakhir: Degasing (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 26) {
            terminalStatus = "Terminal Terakhir: Tikungan Samping Lab (Berangkat)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 35) {
            terminalStatus = "Terminal Terakhir: Jalur 2 Lurus Setelah Beacukai (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 37) {
            terminalStatus = "Terminal Terakhir: Jalur 2 Tikungan Beacukai (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 38) {
            terminalStatus = "Terminal Terakhir: Jalur 2 Lurus Sebelum Beacukai (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 40) {
            terminalStatus = "Terminal Terakhir: Jalur 2 Tikungan Bawah Tangga (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 41) {
            terminalStatus = "Terminal Terakhir: Jalur 2 Lurus Samping Yokai 1 (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }
        else if (terminal_terakhir_value == 43 || terminal_terakhir_value == 48) {
            terminalStatus = "Terminal Terakhir: Jalur 2 Tikungan Jembatan Penyebrangan (Pulang)";
            terminal_terakhir.innerHTML = terminalStatus;
        }

        // if (terminalStatus && terminalStatus !== lastTerminalStatus) {
        //     lastTerminalStatus = terminalStatus;
        //     logCurrentRobotState();
        // }
    });
});

function destroy_ros_all() {
    isTryingToConnect = false;
    is_lap_subscribed = false;
    console.log("Connection to WebSocket server closed.");

    resetBatteryAnimation(0, false);
    updateWiFiWidget(9999);

    if (t2_odometry_subscriber) {
        t2_odometry_subscriber.unsubscribe();
        t2_odometry_subscriber = null;
    }

    if (t2_map_subscriber) {
        t2_map_subscriber.unsubscribe();
        t2_map_subscriber = null;
    }

    if (t2_status_emergency_subscriber) {
        t2_status_emergency_subscriber.unsubscribe();
        t2_status_emergency_subscriber = null;
    }

    if (wp_subscriber) {
        wp_subscriber.unsubscribe();
        wp_subscriber = null;
    }

    if (t2_soc) {
        t2_soc.unsubscribe();
        t2_soc = null;
    }

    if (t2_counter_lap) {
        t2_counter_lap.unsubscribe();
        t2_counter_lap = null;
    }

    if (t2_lag_ms) {
        t2_lag_ms.unsubscribe();
        t2_lag_ms = null;
    }
}

ros.on("error", function (error) {
    destroy_ros_all();
});

ros.on("close", function () {
    destroy_ros_all();
});



// document.addEventListener('keydown', function (event) {
//     if (event.key == "j") {
//         test_audio_play = 1;
//     }
//     else if (event.key == 'u') {
//         test_audio_play = 0;
//     }

// });

// setInterval(() => {
//     if (test_audio_play == 1) {
//         if (alarm.paused) {
//             alarm.play();
//         }
//     }
//     else {
//         if (!alarm.paused) {
//             alarm.pause();
//             alarm.currentTime = 0;
//         }
//     }
// }, 20);
