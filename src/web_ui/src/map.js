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
// var ip_server = window.location.hostname; // IP towing robot
// var ip_server = "127.0.0.1"; // IP towing robot
// var ip_server = "192.168.24.59"; // IP towing robot
var ip_server = "192.168.18.49";
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
    if (!towingimagbiru.complete || !toribeimage.complete) {
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

    let towingimage = null;
    if (ip === "T1") {
        towingimage = towingimagmerah;
    } else if (ip === "T2") {
        towingimage = towingimagbiru;
    } else if (ip === "T3") {
        towingimage = towingimaghijau;
    }

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
        image: towingimagbiru,
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

const towingimagbiru = new Image();
towingimagbiru.src = 'towing_biru.png';

const towingimagmerah = new Image();
towingimagmerah.src = 'towing_merah.png';

const towingimaghijau = new Image();
towingimaghijau.src = 'towing_hijau.png';

const toribeimage = new Image();
toribeimage.src = 'toribe.png';
// ================================================================

let wp_subscriber = null;

let t1_pose_x_packed = 0.0;
let t1_pose_y_packed = 0.0;
let t1_pose_theta_packed = 0.0;
let t1_status_emergency_packed = 0;
let t1_terminal_terakhir_packed = 0;
let t1_battery_soc_packed = 0;
let t1_counter_lap_packed = 0;
let t1_lag_ms_packed = 0;
let t1_lap_sum_packed = 0;

let t2_pose_x_packed = 0.0;
let t2_pose_y_packed = 0.0;
let t2_pose_theta_packed = 0.0;
let t2_status_emergency_packed = 0;
let t2_terminal_terakhir_packed = 0;
let t2_battery_soc_packed = 0;
let t2_counter_lap_packed = 0;
let t2_lag_ms_packed = 0;
let t2_lap_sum_packed = 0;

let t3_pose_x_packed = 0.0;
let t3_pose_y_packed = 0.0;
let t3_pose_theta_packed = 0.0;
let t3_status_emergency_packed = 0;
let t3_terminal_terakhir_packed = 0;
let t3_battery_soc_packed = 0;
let t3_counter_lap_packed = 0;
let t3_lag_ms_packed = 0;
let t3_lap_sum_packed = 0;

let cycleNormal = 0;
let cycleEmergency = 0;

let currentStatus = null;
let lastLoggedStatus = null;
let lastTerminalStatus = null;
let lastRosStatus = null;
let currentLagMs = 0;
let currentSocBat = 0;

let currentSOC = 0;
let currentLap = 0;
let currentX = 0.0;
let currentY = 0.0;
let currentTheta = 0.0;

function checkPriorityStatus(status) {
    let newStatus = null;

    // console.log("Checking status:", status);

    if ((status & STATUS_TOWING_ACTIVE_AUTO) == 0) {
        newStatus = "Towing Mode Manual";
    }
    else if ((status & EMERGENCY_STOP_KARENA_OBSTACLE) == EMERGENCY_STOP_KARENA_OBSTACLE) {
        newStatus = "WARNING: Towing Berhenti Karena Obstacle";
    }
    else if ((status & EMERGENCY_GYRO_ANOMALY_DETECTED) == EMERGENCY_GYRO_ANOMALY_DETECTED) {
        newStatus = "WARNING: Gyro Anomali Terdeteksi";
    }
    else if ((status & EMERGENCY_ICP_SCORE_TERLALU_BESAR) == EMERGENCY_ICP_SCORE_TERLALU_BESAR) {
        newStatus = "WARNING: Anomali Lingkungan Terdeteksi";
    }
    else if ((status & EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR) == EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR) {
        newStatus = "WARNING: Hipotesis Kesalahan Posisi";
    }
    else if ((status & EMERGENCY_GANDENGAN_LEPAS) == EMERGENCY_GANDENGAN_LEPAS) {
        status_emergency.innerHTML = "WARNING: TIDAK ADA TORIBE";
    }
    else if ((status & EMERGENCY_LIDAR_DEPAN_DETECTED) == EMERGENCY_LIDAR_DEPAN_DETECTED) {
        newStatus = "WARNING: Lidar Mendeteksi Objek";
    }
    else if ((status & EMERGENCY_CAMERA_OBS_DETECTED) == EMERGENCY_CAMERA_OBS_DETECTED) {
        newStatus = "WARNING: Kamera Mendeteksi Objek";
    }
    else {
        newStatus = "Towing Normal";
    }

    const isPriority = newStatus !== "Towing Normal" && newStatus !== "Towing Mode Manual";
    return { newStatus, isPriority };
}

function checkTerminalStatus(terminal) {
    let terminalStatus = null;

    // 0 Berangkat: Area jibcrane IST 
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
    if (terminal == -1) {
        terminalStatus = "Terminal Terakhir: Tempat Parkir";
    }
    else if (terminal == 0) {
        terminalStatus = "Terminal Terakhir: Area Jibcrane IST (Berangkat)";
    }
    else if (terminal == 1) {
        terminalStatus = "Terminal Terakhir: Tikungan Samping Yokai 1 (Berangkat)";
    }
    else if (terminal == 3) {
        terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Samping Yokai 1 (Berangkat)";
    }
    else if (terminal == 5 || terminal == 6) {
        terminalStatus = "Terminal Terakhir: Jalur 1 Tikungan Bawah Tangga (Berangkat)";
    }
    else if (terminal == 7) {
        terminalStatus = "Terminal Terakhir: Degasing (Pulang)";
    }
    else if (terminal == 11) {
        terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Setelah Bawah Tangga (Berangkat)";
    }
    else if (terminal == 15) {
        terminalStatus = "Terminal Terakhir: Jalur 1 Tikungan Beacukai (Berangkat)";
    }
    else if (terminal == 19) {
        terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Setelah Beacukai (Berangkat)";
    }
    else if (terminal == 23 || terminal == 46) {
        terminalStatus = "Terminal Terakhir: Degasing (Berangkat)";
    }
    else if (terminal == 24) {
        terminalStatus = "Terminal Terakhir: Jalur 1 Lurus Depan Yokai 1 (Berangkat)";
    }
    else if (terminal == 25 || terminal == 47) {
        terminalStatus = "Terminal Terakhir: Degasing (Pulang)";
    }
    else if (terminal == 26) {
        terminalStatus = "Terminal Terakhir: Tikungan Samping Lab (Berangkat)";
    }
    else if (terminal == 35) {
        terminalStatus = "Terminal Terakhir: Jalur 2 Lurus Setelah Beacukai (Pulang)";
    }
    else if (terminal == 37) {
        terminalStatus = "Terminal Terakhir: Jalur 2 Tikungan Beacukai (Pulang)";
    }
    else if (terminal == 38) {
        terminalStatus = "Terminal Terakhir: Jalur 2 Lurus Sebelum Beacukai (Pulang)";
    }
    else if (terminal == 40) {
        terminalStatus = "Terminal Terakhir: Jalur 2 Tikungan Bawah Tangga (Pulang)";
    }
    else if (terminal == 41) {
        terminalStatus = "Terminal Terakhir: Jalur 2 Lurus Samping Yokai 1 (Pulang)";
    }
    else if (terminal == 43 || terminal == 48) {
        terminalStatus = "Terminal Terakhir: Jalur 2 Tikungan Jembatan Penyebrangan (Pulang)";
    }
    return terminalStatus;
}

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

// CAPEKKKKK, NEXT menambahkan terminal terakhir
ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
    const connectionStatus = "ROS Connected";

    // ===================================================
    t1_packed = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t1/packed',
        messageType: 'ros2_interface/Towing'
    });
    t1_packed.subscribe(function (message) {
        // console.log("T1 Packed message received.");
        t1_pose_x_packed = message.pose_x.data;
        t1_pose_y_packed = message.pose_y.data;
        t1_pose_theta_packed = message.pose_theta.data;
        t1_status_emergency_packed = message.status_emergency.data;
        t1_terminal_terakhir_packed = message.terminal_terakhir.data;
        t1_battery_soc_packed = message.battery_soc.data;
        t1_counter_lap_packed = message.counter_lap.data;
        t1_lag_ms_packed = message.lag_ms.data;
        t1_lap_sum_packed = message.lap_sum.data;
        last_time_connect = new Date();
        // console.log(t1_pose_x_packed, t1_pose_y_packed, t1_pose_theta_packed);
        // console.log(t1_status_emergency_packed, t1_terminal_terakhir_packed, t1_battery_soc_packed);
        // console.log(t1_counter_lap_packed, t1_lag_ms_packed, t1_lap_sum_packed);
    });

    t2_packed = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t2/packed',
        messageType: 'ros2_interface/Towing'
    });
    t2_packed.subscribe(function (message) {
        // console.log("T2 Packed message received.");
        t2_pose_x_packed = message.pose_x.data;
        t2_pose_y_packed = message.pose_y.data;
        t2_pose_theta_packed = message.pose_theta.data;
        t2_status_emergency_packed = message.status_emergency.data;
        t2_terminal_terakhir_packed = message.terminal_terakhir.data;
        t2_battery_soc_packed = message.battery_soc.data;
        t2_counter_lap_packed = message.counter_lap.data;
        t2_lag_ms_packed = message.lag_ms.data;
        t2_lap_sum_packed = message.lap_sum.data;
        last_time_connect = new Date();
        // console.log(t2_pose_x_packed, t2_pose_y_packed, t2_pose_theta_packed);
        // console.log(t2_status_emergency_packed, t2_terminal_terakhir_packed, t2_battery_soc_packed);
        // console.log(t2_counter_lap_packed, t2_lag_ms_packed, t2_lap_sum_packed);
    });

    t3_packed = new ROSLIB.Topic({
        ros: ros,
        name: '/udp/t3/packed',
        messageType: 'ros2_interface/Towing'
    });
    t3_packed.subscribe(function (message) {
        // console.log("T3 Packed message received.");
        t3_pose_x_packed = message.pose_x.data;
        t3_pose_y_packed = message.pose_y.data;
        t3_pose_theta_packed = message.pose_theta.data;
        t3_status_emergency_packed = message.status_emergency.data;
        t3_terminal_terakhir_packed = message.terminal_terakhir.data;
        t3_battery_soc_packed = message.battery_soc.data;
        t3_counter_lap_packed = message.counter_lap.data;
        t3_lag_ms_packed = message.lag_ms.data;
        t3_lap_sum_packed = message.lap_sum.data;
        last_time_connect = new Date();
        // console.log(t3_pose_x_packed, t3_pose_y_packed, t3_pose_theta_packed);
        // console.log(t3_status_emergency_packed, t3_terminal_terakhir_packed, t3_battery_soc_packed);
        // console.log(t3_counter_lap_packed, t3_lag_ms_packed, t3_lap_sum_packed);
    });
    // ===================================================

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
            stroke: 'black',
            strokeWidth: 15,
        });
        waypointsLayer.add(waypointsLine);

        waypointsLayer.draw();

        wp_subscriber.unsubscribe();
    }
    );
});

function destroy_ros_all() {
    isTryingToConnect = false;
    console.log("Connection to WebSocket server closed.");

    resetBatteryAnimation(0, false);
    updateWiFiWidget(9999);
}

ros.on("error", function (error) {
    destroy_ros_all();
});

ros.on("close", function () {
    destroy_ros_all();
});

setInterval(() => {
    // Change data for display based on priority
    let isT1Priority = false;
    let isT2Priority = false;
    let isT3Priority = false;
    let newStatusT1 = null;
    let newStatusT2 = null;
    let newStatusT3 = null;
    let terminalStatusT1 = null;
    let terminalStatusT2 = null;
    let terminalStatusT3 = null;

    ({ newStatus: newStatusT1, isPriority: isT1Priority } = checkPriorityStatus(t1_status_emergency_packed));
    ({ newStatus: newStatusT2, isPriority: isT2Priority } = checkPriorityStatus(t2_status_emergency_packed));
    ({ newStatus: newStatusT3, isPriority: isT3Priority } = checkPriorityStatus(t3_status_emergency_packed));

    // console.log("T1 Priority:", isT1Priority, "Status:", newStatusT1);
    // console.log("T2 Priority:", isT2Priority, "Status:", newStatusT2);
    // console.log("T3 Priority:", isT3Priority, "Status:", newStatusT3);

    terminalStatusT1 = checkTerminalStatus(t1_terminal_terakhir_packed);
    terminalStatusT2 = checkTerminalStatus(t2_terminal_terakhir_packed);
    terminalStatusT3 = checkTerminalStatus(t3_terminal_terakhir_packed);

    let countPriority = 0;
    if (isT1Priority) countPriority++;
    if (isT2Priority) countPriority++;
    if (isT3Priority) countPriority++;

    let listPriority = [];
    if (isT1Priority) listPriority.push(1);
    if (isT2Priority) listPriority.push(2);
    if (isT3Priority) listPriority.push(3);

    if (t1_lag_ms_packed < 1000) {
        addRobotImageWithToribe("T1", t1_pose_x_packed, t1_pose_y_packed, t1_pose_theta_packed, 4.05, 2.0);
    } else {
        addRobotImageWithToribe("T1", 99999, 99999, t1_pose_theta_packed, 4.05, 2.0);
        newStatusT1 = "Towing Disconnected";
        t1_battery_soc_packed = 0;
    }
    if (t2_lag_ms_packed < 1000) {
        addRobotImageWithToribe("T2", t2_pose_x_packed, t2_pose_y_packed, t2_pose_theta_packed, 4.05, 2.0);
    } else {
        addRobotImageWithToribe("T2", 99999, 99999, t2_pose_theta_packed, 4.05, 2.0);
        newStatusT2 = "Towing Disconnected";
        t2_battery_soc_packed = 0;
    }
    if (t3_lag_ms_packed < 1000) {
        addRobotImageWithToribe("T3", t3_pose_x_packed, t3_pose_y_packed, t3_pose_theta_packed, 4.05, 2.0);
    } else {
        addRobotImageWithToribe("T3", 99999, 99999, t3_pose_theta_packed, 4.05, 2.0);
        newStatusT3 = "Towing Disconnected";
        t3_battery_soc_packed = 0;
    }

    console.log("Count Priority:", countPriority);

    if (countPriority === 0) {
        console.log("Normal Cycle");
        if (cycleNormal > 2) cycleNormal = 0;

        if (cycleNormal === 0) {
            currentLap = t1_lap_sum_packed;
            currentStatus = newStatusT1;
            lastTerminalStatus = terminalStatusT1;
            currentLagMs = t1_lag_ms_packed;
            currentSocBat = t1_battery_soc_packed;
        } else if (cycleNormal === 1) {
            currentLap = t2_lap_sum_packed;
            currentStatus = newStatusT2;
            lastTerminalStatus = terminalStatusT2;
            currentLagMs = t2_lag_ms_packed;
            currentSocBat = t2_battery_soc_packed;
        } else if (cycleNormal === 2) {
            currentLap = t3_lap_sum_packed;
            currentStatus = newStatusT3;
            lastTerminalStatus = terminalStatusT3;
            currentLagMs = t3_lag_ms_packed;
            currentSocBat = t3_battery_soc_packed;
        }
        cycleNormal++;
    } else {
        console.log("Emergency Cycle");
        if (cycleEmergency > countPriority - 1) cycleEmergency = 0;

        if (listPriority[cycleEmergency] === 1) {
            currentLap = t1_lap_sum_packed;
            currentStatus = newStatusT1;
            lastTerminalStatus = terminalStatusT1;
            currentLagMs = t1_lag_ms_packed;
            currentSocBat = t1_battery_soc_packed;
        } else if (listPriority[cycleEmergency] === 2) {
            currentLap = t2_lap_sum_packed;
            currentStatus = newStatusT2;
            lastTerminalStatus = terminalStatusT2;
            currentLagMs = t2_lag_ms_packed;
            currentSocBat = t2_battery_soc_packed;
        } else if (listPriority[cycleEmergency] === 3) {
            currentLap = t3_lap_sum_packed;
            currentStatus = newStatusT3;
            lastTerminalStatus = terminalStatusT3;
            currentLagMs = t3_lag_ms_packed;
            currentSocBat = t3_battery_soc_packed;
        }
        cycleEmergency++;
    }

    // ==================================
    //          ALARM BATERAI
    // ==================================
    if (currentSocBat < 5) {
        if (test_audio_play == 0) {
            if (!alarm_30.paused) {
                alarm_30.pause();
                alarm_30.currentTime = 0;
            }
        }
    } else if (currentSocBat < 30) {
        if (alarm_30.paused) {
            alarm_30.play();
        }
    } else if (currentSocBat < 40) {
        if (alarm_30.paused) {
            alarm_30.play();
        }
    } else {
        if (test_audio_play == 0) {
            if (!alarm_30.paused) {
                alarm_30.pause();
                alarm_30.currentTime = 0;
            }
        }
    }

    // ==================================
    //          ALARM WARNING
    // ==================================
    if (currentStatus === "Towing Mode Manual" || currentStatus === "Towing Normal") {
        if (!alarm.paused) {
            alarm.pause();
            alarm.currentTime = 0;
        }
    } else if (currentStatus !== "WARNING: Lidar Mendeteksi Objek" && currentStatus !== "WARNING: Kamera Mendeteksi Objek" && currentStatus !== "Towing Disconnected") {
        if (alarm.paused) {
            alarm.play();
        }
    }

    const formattedLap = currentLap.toString().padStart(2, '0');
    counter_lap.innerHTML = formattedLap;
    label_lap.style.color = 'white';
    counter_lap.style.color = 'white';
    status_emergency.innerHTML = currentStatus;
    terminal_terakhir.innerHTML = lastTerminalStatus;
    updateWiFiWidget(currentLagMs);
    updateBatteryAnimation(currentSocBat, false);
    console.log("Current Status:", currentStatus, "Current Lap:", currentLap, "T:", cycleNormal + 1);

}, 2000);