/**
 * @file app.js
 * @brief Main JavaScript file for the ESP32 Automatic Window Control Interface.
 *
 * Handles fetching sensor data from the ESP32 server and sending control
 * commands back to it.
 */

$(document).ready(function() {

    // --- Element Selectors ---
    const currentHourElement = $('#currentHour');
    const currentModeElement = $('#currentMode');
    const windowOpenPercentageElement = $('#windowOpenPercentage');
    const tempFullyOpenMinInput = $('#tempFullyOpenMin');
    const tempFullyOpenMaxInput = $('#tempFullyOpenMax');
    const ambientTempElement = $('#ambientTemp');
    const lightResistanceElement = $('#lightResistance');
    const dayStateElement = $('#dayState');

    /**
     * @brief Fetches the current hour from the ESP32 server and updates the UI.
     * The ESP32 is expected to have its time synchronized via NTP.
     */
    function fetchAndUpdateHour() {
        $.getJSON('/currentHour.json', function(data) {
            if (data && typeof data.hour !== 'undefined') {
                currentHourElement.text(data.hour);
            } else {
                currentHourElement.text('--:--:--'); // Show placeholder if data is missing
            }
        }).fail(function() {
            console.error("Error: Could not retrieve current hour from ESP32.");
            currentHourElement.text('Error'); // Show error on UI
        });
    }

    /**
     * @brief Fetches the current window open percentage from the ESP32 server and updates the UI.
     * This value is expected to come from a physical potentiometer reading.
     */
    function fetchWindowOpenPercentage() {
        $.getJSON('/windowPercentage.json', function(data) {
            if (data && typeof data.percentage !== 'undefined') {
                windowOpenPercentageElement.text(data.percentage);
            } else {
                windowOpenPercentageElement.text('--'); // Show placeholder if data is missing
            }
        }).fail(function() {
            console.error("Error: Could not retrieve window open percentage from ESP32.");
            windowOpenPercentageElement.text('Error'); // Show error on UI
        });
    }

    /**
     * @brief Fetches sensor readings from the server and updates the UI.
     *
     * This function makes two GET requests to the ESP32:
     * 1. /ambientTemp.json for the NTC thermistor temperature.
     * 2. /lightSensor.json for the photoresistor resistance and day state.
     */
    function updateSensorReadings() {
        // Fetch Ambient Temperature Data
        $.getJSON('/ambientTemp.json', function(data) {
            if (data && typeof data.temp !== 'undefined') {
                // Update the temperature value, formatted to one decimal place.
                ambientTempElement.html(data.temp.toFixed(1));
            } else {
                ambientTempElement.html('--'); // Show placeholder if data is missing
            }
        }).fail(function() {
            console.error("Error: Could not retrieve ambient temperature data.");
            ambientTempElement.html('--'); // Show placeholder on error
        });

        // Fetch Light Sensor Data
        $.getJSON('/lightSensor.json', function(data) {
            if (data) {
                if (typeof data.resistance !== 'undefined') {
                    // Update the light resistance value.
                    lightResistanceElement.text(data.resistance);
                } else {
                    lightResistanceElement.text('--'); // Show placeholder if data is missing
                }
                if (typeof data.dayState !== 'undefined') {
                    // Update the day state (e.g., SUNNY, CLOUDY).
                    dayStateElement.text(data.dayState);
                } else {
                    dayStateElement.text('--'); // Show placeholder if data is missing
                }
            }
        }).fail(function() {
            console.error("Error: Could not retrieve light sensor data.");
            lightResistanceElement.text('--'); // Show placeholder on error
            dayStateElement.text('--'); // Show placeholder on error
        });
    }

    /**
     * @brief Sends the new window control mode to the server via a POST request.
     * @param {string} mode - The control mode to be sent (e.g., 'potentiometer', 'temp_threshold', 'registers').
     */
    function sendControlMode(mode) {
        let modeText = '';
        switch(mode) {
            case 'potentiometer':
                modeText = 'Potentiometer (Manual)';
                break;
            case 'temp_threshold':
                modeText = 'Temp Threshold';
                break;
            case 'registers':
                modeText = 'Registers (Do Nothing Yet)';
                break;
            default:
                modeText = 'Unknown';
        }
        currentModeElement.text(modeText);
        console.log(`Sending control mode to server: ${mode}`);

        $.ajax({
            url: '/setMode.json', // Endpoint for setting mode
            type: 'POST',
            contentType: 'application/json',
            // The ESP32 code is expected to parse JSON: {"mode": "value"}
            data: JSON.stringify({ mode: mode }),
            success: function(response) {
                console.log('Server successfully received control mode.', response);
            },
            error: function(xhr, status, error) {
                console.error('Error sending control mode:', status, error);
            }
        });
    }

    /**
     * @brief Sends the new "fully open" temperature range to the server via a POST request.
     * @param {number} minTemp - The minimum temperature in Celsius for fully open.
     * @param {number} maxTemp - The maximum temperature in Celsius for fully open.
     */
    function sendTempFullyOpenRange(minTemp, maxTemp) {
        console.log(`Sending fully open temperature range to server: Min=${minTemp}°C, Max=${maxTemp}°C`);

        $.ajax({
            url: '/setTempFullyOpenRange.json', // New endpoint for setting temperature range
            type: 'POST',
            contentType: 'application/json',
            // The ESP32 code is expected to parse JSON: {"minTemp": value, "maxTemp": value}
            data: JSON.stringify({ minTemp: minTemp, maxTemp: maxTemp }),
            success: function(response) {
                console.log('Server successfully received temperature range.', response);
            },
            error: function(xhr, status, error) {
                console.error('Error sending temperature range:', status, error);
            }
        });
    }


    // --- Event Listeners ---

    // Control mode buttons
    $('#modePotentiometer').on('click', function() {
        sendControlMode('potentiometer');
    });
    $('#modeTempThreshold').on('click', function() {
        sendControlMode('temp_threshold');
    });
    $('#modeRegisters').on('click', function() {
        sendControlMode('registers');
    });

    // Temperature fully open range inputs
    // Trigger send when either min or max input changes
    $('#tempFullyOpenMin, #tempFullyOpenMax').on('change', function() {
        const minVal = parseFloat(tempFullyOpenMinInput.val());
        const maxVal = parseFloat(tempFullyOpenMaxInput.val());

        if (!isNaN(minVal) && !isNaN(maxVal)) {
            if (minVal <= maxVal) { // Basic validation: min should not be greater than max
                sendTempFullyOpenRange(minVal, maxVal);
            } else {
                console.warn("Min temperature cannot be greater than Max temperature.");
                alert("Warning: Minimum temperature cannot be greater than maximum temperature."); // User feedback
            }
        } else {
            console.warn("Invalid temperature value entered for range.");
            alert("Warning: Please enter valid numbers for both temperature ranges."); // User feedback
        }
    });


    // --- Initialization ---

    // Fetch initial sensor data, hour, and window percentage as soon as the page loads.
    fetchAndUpdateHour();
    fetchWindowOpenPercentage();
    updateSensorReadings();

    // Set an interval to automatically update sensor data, hour, and window percentage.
    // Hour and general sensor readings can be less frequent, window percentage more.
    setInterval(fetchAndUpdateHour, 5000); // Fetch hour from ESP32 every 5 seconds
    setInterval(fetchWindowOpenPercentage, 1000); // Fetch window percentage more frequently for responsiveness
    setInterval(updateSensorReadings, 5000); // Fetch sensor data every 5 seconds
});