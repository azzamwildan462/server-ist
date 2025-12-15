<!DOCTYPE html>
<html>

<head>
    <title>Web UI</title>
    <link rel="stylesheet" href="./bulma.min.css">
    <link rel="stylesheet" href="./index.css">
    <script src="./roslib.min.js"></script>
    <script src="./anime.min.js"></script>
    <script src="./konva.min.js"></script>
    <style>
        .battery-container {
            position: fixed;
            top: 60px;
            right: 15px;
            z-index: 1000;
            background: rgba(0, 0, 0, 0.9);
            padding: 25px;
            /* 75% dari 25px */
            border-radius: 25px;
            /* 75% dari 25px */
            box-shadow: 0 3px 15px rgba(0, 0, 0, 0.3);
            display: flex;
            align-items: center;
            gap: 20px;
            /* 75% dari 20px */
        }

        .battery-info {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
        }

        .battery-icon {
            width: 90px;
            /* 75% dari 120px */
            height: 188px;
            /* 75% dari 250px */
            border: 5px solid #333;
            /* 75% dari 6px */
            border-radius: 19px;
            /* 75% dari 25px */
            position: relative;
            background: #f5f5f5;
        }

        .battery-tip {
            width: 45px;
            /* 75% dari 60px */
            height: 15px;
            /* 75% dari 20px */
            background: #fff;
            position: absolute;
            left: 50%;
            top: -18px;
            /* 75% dari -24px */
            transform: translateX(-50%);
            border-radius: 6px 6px 0 0;
            /* 75% dari 8px */
        }

        .battery-level {
            height: 0%;
            width: 100%;
            border-radius: 11px;
            /* 75% dari 15px */
            transition: height 0.8s ease, background-color 0.8s ease;
            position: absolute;
            bottom: 0;
            overflow: hidden;
        }

        .battery-level::before {
            content: '';
            position: absolute;
            top: -100%;
            left: 0;
            width: 100%;
            height: 100%;
            background: linear-gradient(0deg,
                    transparent,
                    rgba(255, 255, 255, 0.4),
                    transparent);
            animation: batteryShine 2s infinite;
        }

        @keyframes batteryShine {
            0% {
                top: -100%;
            }

            100% {
                top: 100%;
            }
        }

        .battery-percentage {
            color: #fff;
            font-size: 36px;
            /* 75% dari 48px */
            font-weight: bold;
            text-align: center;
            margin-bottom: 8px;
            /* 75% dari 10px */
        }

        /* Battery level colors */
        .battery-high {
            background: linear-gradient(45deg, #4CAF50, #8BC34A);
        }

        .battery-medium {
            background: linear-gradient(45deg, #FF9800, #FFC107);
        }

        .battery-low {
            background: linear-gradient(45deg, #F44336, #E91E63);
            animation: batteryBlink 1s infinite alternate;
        }

        .battery-critical {
            background: linear-gradient(45deg, #B71C1C, #F44336);
            animation: batteryBlink 0.5s infinite alternate;
        }

        @keyframes batteryBlink {
            0% {
                opacity: 1;
            }

            100% {
                opacity: 0.3;
            }
        }

        /* Charging animation */
        .battery-charging {
            animation: batteryPulse 1.5s infinite ease-in-out;
        }

        @keyframes batteryPulse {

            0%,
            100% {
                transform: scale(1);
            }

            50% {
                transform: scale(1.05);
            }
        }

        /* Lightning bolt for charging */
        .charging-bolt {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: #FFD700;
            font-size: 36px;
            /* 75% dari 48px */
            display: none;
            animation: boltFlash 0.8s infinite alternate;
        }

        @keyframes boltFlash {
            0% {
                opacity: 0.5;
            }

            100% {
                opacity: 1;
            }
        }

        .battery-status {
            color: #fff;
            font-size: 14px;
            /* 75% dari 18px */
            text-align: center;
            opacity: 0.8;
        }

        /* LAP Counter Widget */
        .lap-container {
            position: fixed;
            top: 60px;
            right: 285px;
            /* 75% dari 100px */
            z-index: 1000;
            background: rgba(0, 0, 0, 0.8);
            padding: 17px;
            /* 75% dari 25px */
            border-radius: 17px;
            /* 75% dari 25px */
            box-shadow: 0 3px 15px rgba(0, 0, 0, 0.3);
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-width: 113px;
            /* 75% dari 150px */
        }

        .lap-label {
            color: #fff;
            font-size: 2.6rem;
            /* 1.5x dari 1.75rem */
            font-weight: bold;
            text-align: center;
            margin-bottom: 8px;
            /* 75% dari 10px */
        }

        .lap-counter {
            color: #fff;
            font-size: 6rem;
            /* 75% dari 9rem */
            font-weight: bold;
            text-align: center;
        }

        /* WiFi Signal Widget */
        .wifi-container {
            position: fixed;
            top: 60px;
            right: 445px;
            /* 75% dari 380px */
            z-index: 1000;
            background: rgba(0, 0, 0, 0.8);
            padding: 20px;
            /* 75% dari 20px */
            border-radius: 20px;
            /* 75% dari 20px */
            box-shadow: 0 3px 15px rgba(0, 0, 0, 0.3);
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-width: 120px;
            /* 75% dari 120px */
            transform: scale(1.0);
            transform-origin: top right;
        }

        .wifi-label {
            color: #fff;
            font-size: 14px;
            font-weight: bold;
            text-align: center;
            margin-bottom: 8px;
        }

        .wifi-bars {
            position: relative;
            /* Lebarkan kontainer agar semua busur muat */
            width: 70px;
            /* Diperbesar dari 60px */
            height: 50px;
            /* Diperbesar dari 40px */
            margin-bottom: 8px;
            display: flex;
            justify-content: center;
            align-items: flex-end;
        }

        .wifi-bar {
            position: absolute;
            /* Sesuaikan posisi vertikal agar busur tidak terpotong di bagian bawah */
            bottom: -10px;
            /* Geser ke bawah 10px */
            border-radius: 50%;
            border: 4px solid #fff;
            clip-path: inset(0 0 50% 0);
        }

        /* Ukuran berbeda untuk setiap busur agar bertingkat */
        .wifi-bar-1 {
            width: 10px;
            height: 10px;
        }

        .wifi-bar-2 {
            width: 25px;
            height: 25px;
        }

        .wifi-bar-3 {
            width: 40px;
            height: 40px;
        }

        .wifi-bar-4 {
            width: 55px;
            height: 55px;
        }

        /* Logika pewarnaan baru, sekarang hanya mengubah border-color */
        .wifi-no-signal .wifi-bar {
            border-color: #ffffff;
        }

        .wifi-poor .wifi-bar-1 {
            border-color: #F44336;
        }

        .wifi-fair .wifi-bar-1,
        .wifi-fair .wifi-bar-2 {
            border-color: #FF9800;
        }

        .wifi-good .wifi-bar-1,
        .wifi-good .wifi-bar-2,
        .wifi-good .wifi-bar-3 {
            border-color: #f3e307;
        }

        .wifi-excellent .wifi-bar-1,
        .wifi-excellent .wifi-bar-2,
        .wifi-excellent .wifi-bar-3,
        .wifi-excellent .wifi-bar-4 {
            border-color: #4CAF50;
        }

        .custom-max {
            max-width: 20px;
            margin-inline: auto;
        }

        .cam-fixed {
            position: fixed;
            top: 60px;
            left: 15px;
            z-index: 1000;
            width: 640px;
            height: 360px;
            /* keeps shape */
            object-fit: cover;
            border-radius: 10px;
            box-shadow: 0 6px 18px rgba(0, 0, 0, .2);
            background: #111;
            /* placeholder while loading */
        }

        /* hard cap + center, with padding for small screens */
        .center-wrap {
            width: 100%;
            max-width: 600px;
            /* ← adjust this */
            margin-inline: auto;
            /* center horizontally */
            padding-inline: 1rem;
            /* keep breathing room on mobile */
            overflow-wrap: anywhere;
            /* prevent long text from stretching width */
        }

        .date-container {
            position: fixed;
            top: 10px;
            right: 15px;
            /* Sesuaikan posisi ini agar tidak tumpang tindih */
            z-index: 1000;
            background: rgba(0, 0, 0, 0.8);
            padding: 10px 20px;
            border-radius: 10px;
            box-shadow: 0 3px 15px rgba(0, 0, 0, 0.3);
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .time-container {
            position: fixed;
            top: 10px;
            right: 285px;
            z-index: 1000;
            background: rgba(0, 0, 0, 0.8);
            padding: 10px 20px;
            border-radius: 10px;
            box-shadow: 0 3px 15px rgba(0, 0, 0, 0.3);
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .date-text {
            color: #fff;
            font-size: 1rem;
            /* Ukuran font 16px */
            font-weight: bold;
            text-align: center;
            white-space: nowrap;
            /* Mencegah teks pindah baris */
        }
    </style>
</head>

<body>
    <nav class="navbar" role="navigation" aria-label="main navigation">
        <div class="navbar-brand">
            <a class="navbar-item" href="https://bulma.io">
                <svg width="640" height="160" viewBox="0 0 640 160" fill="none" xmlns="http://www.w3.org/2000/svg"
                    class="navbar-svgs">
                    <path stroke="#3498db" stroke-width="10" fill="none"
                        d="M10,150 L100,150 L120,40 L140,150 L130,95 L110,95 L109,100 L135,100" fill="red"
                        class="bd-svg-black" />
                    <!-- A -->
                    <path stroke="#3498db" stroke-width="10" fill="none" d="M140,150 L160,40 L180,150 L200,40 L400,40"
                        fill="red" class="bd-svg-black" /> <!-- W -->
                </svg>
            </a>

            <a role="button" class="navbar-burger" aria-label="menu" aria-expanded="false"
                data-target="navbarBasicExample">
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
                <span aria-hidden="true"></span>
            </a>
        </div>
    </nav>

    <div class="date-container">
        <div id="date-display" class="date-text"></div>
    </div>
    <div class="time-container">
        <div id="time-display" class="date-text"></div>
    </div>

    <!-- Battery Animation Widget -->
    <div class="battery-container">
        <div class="battery-info">
            <div class="battery-percentage" id="battery-percentage">0%</div>
            <div class="battery-status" id="battery-status">Disconnected</div>
        </div>
        <div class="battery-icon" id="battery-icon">
            <div class="battery-level" id="battery-level"></div>
            <div class="battery-tip"></div>
            <div class="charging-bolt" id="charging-bolt">⚡</div>
        </div>
    </div>

    <!-- LAP Counter Widget -->
    <div class="lap-container">
        <div class="lap-label" id="label-lap">LAP</div>
        <div class="lap-counter" id="counter-lap">00</div>
    </div>

    <!-- WiFi Signal Widget -->
    <div class="wifi-container" id="wifi-container">
        <div class="wifi-label">WiFi</div>
        <div class="wifi-bars">
            <div class="wifi-bar wifi-bar-1" id="wifi-bar-1"></div>
            <div class="wifi-bar wifi-bar-2" id="wifi-bar-2"></div>
            <div class="wifi-bar wifi-bar-3" id="wifi-bar-3"></div>
            <div class="wifi-bar wifi-bar-4" id="wifi-bar-4"></div>
        </div>
        <div class="wifi-status" id="wifi-status">--ms</div>
    </div>

    <!-- Camera Feed -->
    <img id="cam-main" class="cam-fixed" src="" />

    <br>

    <div class="container is-fluid">
        <!-- Kata Kata -->
        <section class="section is-flex is-justify-content-center is-align-items-center p-0"
            style="min-height: calc(10dvh - 3.25rem);">
            <div class="center-wrap has-text-centered">
                <div id="towing-id" class="has-text-weight-bold is-size-1">
                    Towing: Disconnected
                </div>
            </div>
        </section>
        <section class="section is-flex is-justify-content-center is-align-items-center p-0"
            style="min-height: calc(10dvh - 3.25rem);">
            <div class="center-wrap has-text-centered">
                <div id="status-emergency" class="has-text-weight-bold is-size-3">
                    WARNING: Towing Disconnected
                </div>
            </div>
        </section>
        <section class="section is-flex is-justify-content-center is-align-items-center p-0"
            style="min-height: calc(10dvh - 3.25rem);">
            <div class="center-wrap has-text-centered">
                <div id="terminal-terakhir" class="has-text-weight-bold is-size-3">
                    Terminal Terakhir:
                </div>
            </div>
        </section>


        <div id="map"></div>
    </div>

    <script type="text/javascript" src="map.js"></script>
</body>

</html>