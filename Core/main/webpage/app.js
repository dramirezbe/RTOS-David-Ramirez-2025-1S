/**
 * @file app.js
 * @brief This file contains the JavaScript logic for the ESP32 Javastral web interface.
 * It handles fetching NTC sensor data and sending RGB LED values to the ESP32.
 */

/**
 * Global variables (currently not used in this simplified version, but kept for potential future use)
 */
var seconds = null;
var otaTimerVar = null;
var wifiConnectInterval = null;
var uart_on = true;

/**
 * Initializes functions when the DOM is ready.
 */
$(document).ready(function() {
    // getUpdateStatus(); // Commented out: OTA functionality not currently used
    startNTCsensorInterval(); // Start fetching NTC sensor values
    
    // Activate listeners for RGB input fields to restrict values
    activate_listener('red_val');
    activate_listener('green_val');
    activate_listener('blue_val');

    // Activate listeners for temperature threshold input fields
    activate_listener('red_min');
    activate_listener('red_max');
    activate_listener('green_min');
    activate_listener('green_max');
    activate_listener('blue_min');
    activate_listener('blue_max');
    
    // Attach click event handlers for the RGB and LED control buttons
    $("#send_rgb").on("click", function() {
        send_rgb_values();
    });
    $("#toogle_led").on("click", function() {
        toogle_led();
    });

    // Attach click event handler for the temperature threshold button
    $("#send_temp_threshold").on("click", function() {
        send_temp_threshold();
    });

    // Initialize UART button state and attach click handler
    const uartButton = $("#apagar_uart");
    if (uart_on) {
        uartButton.val("UART On").css("background-color", "green");
    } else {
        uartButton.val("UART Off").css("background-color", "red");
    }
    uartButton.on("click", function(){
        toggle_uart();
    });
    
    // $("#send_temp_threshold").on("click", function(){ // Commented out: Temperature threshold functionality not implemented yet
    //     // add functionality for temp threshold
    // });
});

/**
 * Commented out: Functions related to OTA firmware update, as they are not currently in use.
 *
 * function getFileInfo() {
 * var x = document.getElementById("selected_file");
 * var file = x.files[0];
 * document.getElementById("file_info").innerHTML = "<h4>File: " + file.name + "<br>" + "Size: " + file.size + " bytes</h4>";
 * }
 *
 * function updateFirmware() {
 * var formData = new FormData();
 * var fileSelect = document.getElementById("selected_file");
 * if (fileSelect.files && fileSelect.files.length == 1) {
 * var file = fileSelect.files[0];
 * formData.set("file", file, file.name);
 * document.getElementById("ota_update_status").innerHTML = "Uploading " + file.name + ", Firmware Update in Progress...";
 * var request = new XMLHttpRequest();
 * request.upload.addEventListener("progress", updateProgress);
 * request.open('POST', "/OTAupdate");
 * request.responseType = "blob";
 * request.send(formData);
 * } else {
 * window.alert('Select A File First')
 * }
 * }
 *
 * function updateProgress(oEvent) {
 * if (oEvent.lengthComputable) {
 * getUpdateStatus();
 * } else {
 * window.alert('total size is unknown')
 * }
 * }
 *
 * function getUpdateStatus() {
 * var xhr = new XMLHttpRequest();
 * var requestURL = "/OTAstatus";
 * xhr.open('POST', requestURL, false);
 * xhr.send('ota_update_status');
 * if (xhr.readyState == 4 && xhr.status == 200) {
 * var response = JSON.parse(xhr.responseText);
 * document.getElementById("latest_firmware").innerHTML = response.compile_date + " - " + response.compile_time
 * if (response.ota_update_status == 1) {
 * seconds = 10;
 * otaRebootTimer();
 * } else if (response.ota_update_status == -1) {
 * document.getElementById("ota_update_status").innerHTML = "!!! Upload Error !!!";
 * }
 * }
 * }
 *
 * function otaRebootTimer() {
 * document.getElementById("ota_update_status").innerHTML = "OTA Firmware Update Complete. This page will close shortly, Rebooting in: " + seconds;
 * if (--seconds == 0) {
 * clearTimeout(otaTimerVar);
 * window.location.reload();
 * } else {
 * otaTimerVar = setTimeout(otaRebootTimer, 1000);
 * }
 * }
 */

/**
 * Gets NTC sensor temperature value for display on the web page.
 * The endpoint is `/ntcSensor.json`, but we are assuming it now returns NTC data.
 */
function getNTCsensorValues() {
    $.getJSON('/ntcSensor.json', function(data) {
        // Assuming the JSON response contains a 'temp' field for the temperature reading
        $("#temperature_reading").text(data["temp"] + " °C"); // Added unit for clarity
    });
}

/**
 * Sets the interval for getting the updated NTC sensor values.
 */
function startNTCsensorInterval() {
    setInterval(getNTCsensorValues, 5000); // Fetch temperature every 5 seconds
}

/**
 * Commented out: Functions related to WiFi connection status, as they are not currently in use.
 *
 * function stopWifiConnectStatusInterval() {
 * if (wifiConnectInterval != null) {
 * clearInterval(wifiConnectInterval);
 * wifiConnectInterval = null;
 * }
 * }
 *
 * function getWifiConnectStatus() {
 * var xhr = new XMLHttpRequest();
 * var requestURL = "/wifiConnectStatus";
 * xhr.open('POST', requestURL, false);
 * xhr.send('wifi_connect_status');
 * if (xhr.readyState == 4 && xhr.status == 200) {
 * var response = JSON.parse(xhr.responseText);
 * document.getElementById("wifi_connect_status").innerHTML = "Connecting...";
 * if (response.wifi_connect_status == 2) {
 * document.getElementById("wifi_connect_status").innerHTML = "<h4 class='rd'>Failed to Connect. Please check your AP credentials and compatibility</h4>";
 * stopWifiConnectStatusInterval();
 * } else if (response.wifi_connect_status == 3) {
 * document.getElementById("wifi_connect_status").innerHTML = "<h4 class='gr'>Connection Success!</h4>";
 * stopWifiConnectStatusInterval();
 * }
 * }
 * }
 *
 * function startWifiConnectStatusInterval() {
 * wifiConnectInterval = setInterval(getWifiConnectStatus, 2800);
 * }
 */

/**
 * Sends the RGB values to the ESP32.
 * The values are read from the input fields and sent as a JSON object.
 */ 
function send_rgb_values() {
    // Get values from the input fields
    var red_val = $("#red_val").val();
    var green_val = $("#green_val").val();
    var blue_val = $("#blue_val").val();

    // Create a data object to be sent as JSON
    var rgb_data = {
        'red_val': parseInt(red_val),   // Ensure values are integers
        'green_val': parseInt(green_val),
        'blue_val': parseInt(blue_val),
        'timestamp': Date.now()         // Add a timestamp
    };

    // Send the RGB data using AJAX POST request
    $.ajax({
        url: '/rgb_values.json',         // Endpoint for RGB values
        contentType: 'application/json', // Specify that you're sending JSON
        method: 'POST',
        cache: false,
        data: JSON.stringify(rgb_data)   // Convert the data object to a JSON string
    }).done(function() {
        console.log("RGB values sent successfully!");
    }).fail(function(jqXHR, textStatus, errorThrown) {
        console.error("Error sending RGB values:", textStatus, errorThrown);
    });
}

/**
 * Sends the temperature threshold values to the ESP32.
 * The values are read from the input fields and sent as a JSON object.
 */
function send_temp_threshold() {
    // Get values from the input fields for each color's min/max thresholds
    var red_min = $("#red_min").val();
    var red_max = $("#red_max").val();
    var green_min = $("#green_min").val();
    var green_max = $("#green_max").val();
    var blue_min = $("#blue_min").val();
    var blue_max = $("#blue_max").val();

    // Create a data object to be sent as JSON
    var temp_threshold_data = {
        'red_min': parseInt(red_min),
        'red_max': parseInt(red_max),
        'green_min': parseInt(green_min),
        'green_max': parseInt(green_max),
        'blue_min': parseInt(blue_min),
        'blue_max': parseInt(blue_max),
        'timestamp': Date.now() // Add a timestamp for tracking
    };

    // Send the temperature threshold data using AJAX POST request
    $.ajax({
        url: '/temp_threshold.json', // Define your endpoint for temperature thresholds
        contentType: 'application/json', // Specify that you're sending JSON
        method: 'POST',
        cache: false,
        data: JSON.stringify(temp_threshold_data) // Convert the data object to a JSON string
    }).done(function() {
        console.log("Temperature threshold values sent successfully!");
    }).fail(function(jqXHR, textStatus, errorThrown) {
        console.error("Error sending temperature threshold values:", textStatus, errorThrown);
    });
}

function toggle_uart() {
    uart_on = !uart_on; // Toggle the boolean flag
    const uartButton = $("#apagar_uart");

    if (uart_on) {
        uartButton.val("UART On").css("background-color", "green");
        console.log("UART is ON");
    } else {
        uartButton.val("UART Off").css("background-color", "red");
        console.log("UART is OFF");
    }

    // Send the UART status to the ESP32
    $.ajax({
        url: '/toogle_uart.json',
        contentType: 'application/json',
        method: 'POST',
        cache: false,
        data: JSON.stringify({ 'uart_on': uart_on }) // Send the boolean status
    }).done(function() {
        console.log("UART status sent successfully: " + uart_on);
    }).fail(function(jqXHR, textStatus, errorThrown) {
        console.error("Error sending UART status:", textStatus, errorThrown);
    });
}

/**
 * Toggles the board LED by sending a POST request to the ESP32.
 */
function toogle_led() {
    $.ajax({
        url: '/toogle_led.json', // Endpoint to toggle the LED
        dataType: 'json',
        method: 'POST',
        cache: false,
    }).done(function() {
        console.log("LED toggle request sent successfully!");
    }).fail(function(jqXHR, textStatus, errorThrown) {
        console.error("Error toggling LED:", textStatus, errorThrown);
    });
}

/**
 * Activates an input listener for specified element IDs.
 * Ensures the input value is an integer between 0 and 255.
 * @param {string} used_id The ID of the input element to listen to.
 */
function activate_listener(used_id) {
    const myInput = document.getElementById(used_id);

    myInput.addEventListener('input', () => {
        // Convert the input value to a number
        let value = Number(myInput.value);

        // Ensure the value is an integer
        if (!Number.isInteger(value)) {
            value = Math.floor(value);
        }

        // Restrict the value to be between 0 and 255
        if (value > 100) {
            value = 100;
        } else if (value < 0) { // Added a lower bound check
            value = 0;
        }
        myInput.value = value;
    });
}