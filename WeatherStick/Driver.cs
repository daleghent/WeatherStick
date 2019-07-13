//tabs=4
/*
 * Copyright 2019 Dale Ghent <daleg@elemental.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#define ObservingConditions

using ASCOM.DeviceInterface;
using ASCOM.Utilities;
using System;
using System.Collections;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;

namespace ASCOM.WeatherStick {
    //
    // Your driver's DeviceID is ASCOM.WeatherStick.ObservingConditions
    //
    // The Guid attribute sets the CLSID for ASCOM.WeatherStick.ObservingConditions
    // The ClassInterface/None addribute prevents an empty interface called
    // _WeatherStick from being created and used as the [default] interface
    //
    // TODO Replace the not implemented exceptions with code to implement the function or
    // throw the appropriate ASCOM exception.
    //

    /// <summary>
    /// ASCOM ObservingConditions Driver for WeatherStick.
    /// </summary>
    [Guid("19691e43-4d2a-4810-99d2-89ca8137f1f4")]
    [ClassInterface(ClassInterfaceType.None)]
    public class ObservingConditions : IObservingConditions {
        /// <summary>
        /// ASCOM DeviceID (COM ProgID) for this driver.
        /// The DeviceID is used by ASCOM applications to load the driver at runtime.
        /// </summary>
        internal static string driverID = "ASCOM.WeatherStick.ObservingConditions";
        /// <summary>
        /// Driver description that displays in the ASCOM Chooser.
        /// </summary>
        private static string driverDescription = "Weather Stick";

        internal static string comPortProfileName = "COM Port"; // Constants used for Profile persistence
        internal static string comPortDefault = "COM1";
        internal static string traceStateProfileName = "Trace Level";
        internal static string traceStateDefault = "true";

        internal static string queryIntervalProfileName = "Device query interval";
        internal static string queryIntervalDefault = "300";

        // Variables to hold the currrent device configuration
        internal static string comPort;
        internal static double queryInterval;

        /// <summary>
        /// Private variable to hold the connected state
        /// </summary>
        private bool connectedState;

        /// <summary>
        /// Private variable to hold an ASCOM Utilities object
        /// </summary>
        private Util utilities;

        /// <summary>
        /// Variable to hold the trace logger object (creates a diagnostic log file with information that you specify)
        /// </summary>
        internal static TraceLogger tl;

        /// <summary>
        /// Private variable to hold an ASCOM Serial object
        /// </summary>
        private Serial serialConnection;

        // Local variables to hold sensor values
        private double _dewpoint;
        private DateTime _dewpointUpdated;

        private double _humidity;
        private DateTime _humidityUpdated;

        private double _pressure;
        private DateTime _pressureUpdated;

        private double _temperature;
        private DateTime _temperatureUpdated;

        private DateTime _lastUpdated;

        // Cancellation tokens for the hardware query thread
        private Task queryThreadTask;
        private CancellationTokenSource queryThreadCts;
        private CancellationToken ct;

        /// <summary>
        /// Initializes a new instance of the <see cref="WeatherStick"/> class.
        /// Must be public for COM registration.
        /// </summary>
        public ObservingConditions() {
            tl = new TraceLogger("", "WeatherStick");
            ReadProfile(); // Read device configuration from the ASCOM Profile store

            tl.LogMessage("ObservingConditions", "Starting initialisation");

            connectedState = false; // Initialise connected to false
            utilities = new Util();

            tl.LogMessage("ObservingConditions", $"COM port is {comPort}");
            tl.LogMessage("ObservingConditions", "Completed initialisation");
        }


        //
        // PUBLIC COM INTERFACE IObservingConditions IMPLEMENTATION
        //

        #region Common properties and methods.

        /// <summary>
        /// Displays the Setup Dialog form.
        /// If the user clicks the OK button to dismiss the form, then
        /// the new settings are saved, otherwise the old values are reloaded.
        /// THIS IS THE ONLY PLACE WHERE SHOWING USER INTERFACE IS ALLOWED!
        /// </summary>
        public void SetupDialog() {
            if (IsConnected)
                System.Windows.Forms.MessageBox.Show("Already connected, just press OK");

            using (SetupDialogForm F = new SetupDialogForm()) {
                var result = F.ShowDialog();
                if (result == System.Windows.Forms.DialogResult.OK) {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }

        public ArrayList SupportedActions {
            get {
                tl.LogMessage("SupportedActions Get", "Returning empty arraylist");
                return new ArrayList();
            }
        }

        public string Action(string actionName, string actionParameters) {
            LogMessage("", $"Action {actionName}, parameters {actionParameters} not implemented");
            throw new ASCOM.ActionNotImplementedException("Action " + actionName + " is not implemented by this driver");
        }

        public void CommandBlind(string command, bool raw) {
            throw new ASCOM.MethodNotImplementedException("CommandBlind");
        }

        public bool CommandBool(string command, bool raw) {
            throw new ASCOM.MethodNotImplementedException("CommandBool");
        }

        public string CommandString(string command, bool raw) {
            throw new ASCOM.MethodNotImplementedException("CommandString");
        }

        public void Dispose() {
            // Clean up the tracelogger and util objects
            tl.Enabled = false;
            tl.Dispose();
            tl = null;
            utilities.Dispose();
            utilities = null;
            serialConnection.Dispose();
            serialConnection = null;

            queryThreadCts.Dispose();
            queryThreadTask.Dispose();
        }

        public bool Connected {
            get => IsConnected;
            set {
                tl.LogMessage("Connected", $"Set {value}");
                if (value == IsConnected)
                    return;

                if (value) {
                    LogMessage("Connected Set", $"Connecting to port {comPort}");

                    ConnectWeatherStick();
                    connectedState = true;

                    queryThreadCts = new CancellationTokenSource();
                    ct = queryThreadCts.Token;
                    queryThreadTask = QueryDevice();
                } else {
                    LogMessage("Connected Set", $"Disconnecting from port {comPort}");
                    connectedState = false;

                    queryThreadCts.Cancel();

                    DisconnectWeatherStick();
                    Dispose();
                }
            }
        }

        public string Description {
            get {
                tl.LogMessage("Description Get", driverDescription);
                return driverDescription;
            }
        }

        public string DriverInfo {
            get {
                string driverInfo = "Driver for the Arduino-based Weather Stick";
                tl.LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        public string DriverVersion {
            get {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        public short InterfaceVersion {
            // set by the driver wizard
            get {
                LogMessage("InterfaceVersion Get", "1");
                return Convert.ToInt16("1");
            }
        }

        public string Name {
            get {
                string name = "WeatherStick";
                tl.LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion

        #region IObservingConditions Implementation

        /// <summary>
        /// Gets and sets the time period over which observations wil be averaged
        /// </summary>
        /// <remarks>
        /// Get must be implemented, if it can't be changed it must return 0
        /// Time period (hours) over which the property values will be averaged 0.0 =
        /// current value, 0.5= average for the last 30 minutes, 1.0 = average for the
        /// last hour
        /// </remarks>
        public double AveragePeriod {
            get => 0;
            set {
                if (value != 0) {
                    throw new InvalidValueException("AveragePeriod", value.ToString(), "0 only");
                }
            }
        }

        /// <summary>
        /// Amount of sky obscured by cloud
        /// </summary>
        /// <remarks>0%= clear sky, 100% = 100% cloud coverage</remarks>
        public double CloudCover => throw new PropertyNotImplementedException("CloudCover", false);

        /// <summary>
        /// Atmospheric dew point at the observatory in deg C
        /// </summary>
        /// <remarks>
        /// Normally optional but mandatory if <see cref=" ASCOM.DeviceInterface.IObservingConditions.Humidity"/>
        /// Is provided
        /// </remarks>
        public double DewPoint {
            get {
                LogMessage("DewPoint", "get");
                return _dewpoint;
            }
            private set => _dewpoint = value;
        }

        /// <summary>
        /// Atmospheric relative humidity at the observatory in percent
        /// </summary>
        /// <remarks>
        /// Normally optional but mandatory if <see cref="ASCOM.DeviceInterface.IObservingConditions.DewPoint"/> 
        /// Is provided
        /// </remarks>
        public double Humidity {
            get {
                LogMessage("Humidity", "get");
                return _humidity;
            }
            private set => _humidity = value;
        }

        /// <summary>
        /// Atmospheric pressure at the observatory in hectoPascals (mB)
        /// </summary>
        /// <remarks>
        /// This must be the pressure at the observatory and not the "reduced" pressure
        /// at sea level. Please check whether your pressure sensor delivers local pressure
        /// or sea level pressure and adjust if required to observatory pressure.
        /// </remarks>
        public double Pressure {
            get {
                LogMessage("Pressure", "get");
                return _pressure;
            }
            private set => _pressure = value;
        }

        /// <summary>
        /// Rain rate at the observatory
        /// </summary>
        /// <remarks>
        /// This property can be interpreted as 0.0 = Dry any positive nonzero value
        /// = wet.
        /// </remarks>
        public double RainRate => throw new PropertyNotImplementedException("RainRate", false);

        /// <summary>
        /// Forces the driver to immediatley query its attached hardware to refresh sensor
        /// values
        /// </summary>
        public void Refresh() {
            throw new ASCOM.MethodNotImplementedException();
        }

        /// <summary>
        /// Provides a description of the sensor providing the requested property
        /// </summary>
        /// <param name="PropertyName">Name of the property whose sensor description is required</param>
        /// <returns>The sensor description string</returns>
        /// <remarks>
        /// PropertyName must be one of the sensor properties, 
        /// properties that are not implemented must throw the MethodNotImplementedException
        /// </remarks>
        public string SensorDescription(string PropertyName) {
            switch (PropertyName.Trim().ToLowerInvariant()) {
                case "averageperiod":
                    return "Average period in hours, immediate values are only available";
                case "dewpoint":
                    return "The dew point calculated using humidity and temperature";
                case "humidity":
                    return "The current relative humidity - measured by DHT22";
                case "pressure":
                    return "The current air pressure in hectopascals - measured by BMP280";
                case "temperature":
                    return "The current air temperature in degrees celcius - measured by DHT22";
                case "cloudcover":
                case "rainrate":
                case "skybrightness":
                case "skyquality":
                case "starfwhm":
                case "skytemperature":
                case "winddirection":
                case "windgust":
                case "windspeed":
                    LogMessage("SensorDescription", "{0} - not implemented", PropertyName);
                    throw new MethodNotImplementedException("SensorDescription(" + PropertyName + ")");
                default:
                    LogMessage("SensorDescription", "{0} - unrecognised", PropertyName);
                    throw new ASCOM.InvalidValueException("SensorDescription(" + PropertyName + ")");
            }
        }

        /// <summary>
        /// Sky brightness at the observatory
        /// </summary>
        public double SkyBrightness => throw new PropertyNotImplementedException("SkyBrightness", false);

        /// <summary>
        /// Sky quality at the observatory
        /// </summary>
        public double SkyQuality => throw new PropertyNotImplementedException("SkyQuality", false);

        /// <summary>
        /// Seeing at the observatory
        /// </summary>
        public double StarFWHM => throw new PropertyNotImplementedException("StarFWHM", false);

        /// <summary>
        /// Sky temperature at the observatory in deg C
        /// </summary>
        public double SkyTemperature => throw new PropertyNotImplementedException("SkyTemperature", false);

        /// <summary>
        /// Temperature at the observatory in deg C
        /// </summary>
        public double Temperature {
            get {
                LogMessage("Temperature", "get");
                return _temperature;
            }
            private set => _temperature = value;
        }

        /// <summary>
        /// Provides the time since the sensor value was last updated
        /// </summary>
        /// <param name="PropertyName">Name of the property whose time since last update Is required</param>
        /// <returns>Time in seconds since the last sensor update for this property</returns>
        /// <remarks>
        /// PropertyName should be one of the sensor properties Or empty string to get
        /// the last update of any parameter. A negative value indicates no valid value
        /// ever received.
        /// </remarks>
        public double TimeSinceLastUpdate(string PropertyName) {
            if (!string.IsNullOrEmpty(PropertyName)) {
                switch (PropertyName.Trim().ToLowerInvariant()) {
                    case "dewpoint":
                        return SecondsSince(_dewpointUpdated);
                    case "humidity":
                        return SecondsSince(_humidityUpdated);
                    case "pressure":
                        return SecondsSince(_pressureUpdated);
                    case "temperature":
                        return SecondsSince(_temperatureUpdated);

                    case "averageperiod":
                    case "cloudcover":
                    case "rainrate":
                    case "skybrightness":
                    case "skyquality":
                    case "starfwhm":
                    case "skytemperature":
                    case "winddirection":
                    case "windgust":
                    case "windspeed":
                        LogMessage("TimeSinceLastUpdate", "{0} - not implemented", PropertyName);
                        throw new MethodNotImplementedException("SensorDescription(" + PropertyName + ")");
                    default:
                        LogMessage("TimeSinceLastUpdate", "{0} - unrecognised", PropertyName);
                        throw new ASCOM.InvalidValueException("SensorDescription(" + PropertyName + ")");
                }
            }

            return SecondsSince(_lastUpdated);
        }

        /// <summary>
        /// Wind direction at the observatory in degrees
        /// </summary>
        /// <remarks>
        /// 0..360.0, 360=N, 180=S, 90=E, 270=W. When there Is no wind the driver will
        /// return a value of 0 for wind direction
        /// </remarks>
        public double WindDirection => throw new PropertyNotImplementedException("WindDirection", false);

        /// <summary>
        /// Peak 3 second wind gust at the observatory over the last 2 minutes in m/s
        /// </summary>
        public double WindGust => throw new PropertyNotImplementedException("WindGust", false);

        /// <summary>
        /// Wind speed at the observatory in m/s
        /// </summary>
        public double WindSpeed => throw new PropertyNotImplementedException("WindSpeed", false);

        #endregion

        #region private methods

        private async Task QueryDevice() {
            try {
                LogMessage("QueryDevice", "Starting device query thread");

                while (true) {
                    DewPoint = WSCommand("DEW#");
                    _dewpointUpdated = DateTime.Now;
                    ct.ThrowIfCancellationRequested();

                    Humidity = WSCommand("HUM#");
                    _humidityUpdated = DateTime.Now;
                    ct.ThrowIfCancellationRequested();

                    Pressure = WSCommand("PRE#");
                    _pressureUpdated = DateTime.Now;
                    ct.ThrowIfCancellationRequested();

                    Temperature = WSCommand("TEM#");
                    _temperatureUpdated = DateTime.Now;
                    ct.ThrowIfCancellationRequested();

                    _lastUpdated = DateTime.Now;

                    await Task.Delay(TimeSpan.FromSeconds(queryInterval), ct);
                }
            } catch (OperationCanceledException) {
                LogMessage("QueryDevice", "Thread cancelled via token");
            }
        }

        private void ConnectWeatherStick() {
            try {
                serialConnection = new Serial {
                    PortName = comPort,
                    Speed = SerialSpeed.ps57600
                };

                serialConnection.Connected = true;
                utilities.WaitForMilliseconds(2000);
                serialConnection.ClearBuffers();

                connectedState = true;
            } catch (Exception ex) {
                throw new DriverException($"Failed attempting to connect to {comPort}", ex);
            }
        }

        private void DisconnectWeatherStick() {
            connectedState = false;
            serialConnection.Connected = false;
        }

        private double WSCommand(string command) {
            double value;
            string response;

            try {
                if (IsConnected) {
                    LogMessage("WSCommand", $"Command: {command}");

                    serialConnection.ClearBuffers();

                    serialConnection.Transmit("#");

                    utilities.WaitForMilliseconds(500);
                    ct.ThrowIfCancellationRequested();

                    serialConnection.Transmit(command);
                    ct.ThrowIfCancellationRequested();

                    response = serialConnection.ReceiveTerminated("#");
                    ct.ThrowIfCancellationRequested();

                    LogMessage("WSCommand", $"Command: {command}, Response: {response}");

                    value = Convert.ToDouble(response.TrimEnd('#'));
                } else {
                    return double.NaN;
                }

            } catch (Exception ex) {
                if (ex is OperationCanceledException) {
                    throw new DriverException("WSCommand cancelled", ex);
                } else {
                    throw new DriverException($"Failed to communicate with WeatherStick on {comPort}", ex);
                }
            }

            return value;
        }
        #endregion

        #region Private properties and methods
        // here are some useful properties and methods that can be used as required
        // to help with driver development

        #region ASCOM Registration

        // Register or unregister driver for ASCOM. This is harmless if already
        // registered or unregistered. 
        //
        /// <summary>
        /// Register or unregister the driver with the ASCOM Platform.
        /// This is harmless if the driver is already registered/unregistered.
        /// </summary>
        /// <param name="bRegister">If <c>true</c>, registers the driver, otherwise unregisters it.</param>
        private static void RegUnregASCOM(bool bRegister) {
            using (var P = new ASCOM.Utilities.Profile()) {
                P.DeviceType = "ObservingConditions";
                if (bRegister) {
                    P.Register(driverID, driverDescription);
                } else {
                    P.Unregister(driverID);
                }
            }
        }

        /// <summary>
        /// This function registers the driver with the ASCOM Chooser and
        /// is called automatically whenever this class is registered for COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is successfully built.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During setup, when the installer registers the assembly for COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually register a driver with ASCOM.
        /// </remarks>
        [ComRegisterFunction]
        public static void RegisterASCOM(Type t) {
            RegUnregASCOM(true);
        }

        /// <summary>
        /// This function unregisters the driver from the ASCOM Chooser and
        /// is called automatically whenever this class is unregistered from COM Interop.
        /// </summary>
        /// <param name="t">Type of the class being registered, not used.</param>
        /// <remarks>
        /// This method typically runs in two distinct situations:
        /// <list type="numbered">
        /// <item>
        /// In Visual Studio, when the project is cleaned or prior to rebuilding.
        /// For this to work correctly, the option <c>Register for COM Interop</c>
        /// must be enabled in the project settings.
        /// </item>
        /// <item>During uninstall, when the installer unregisters the assembly from COM Interop.</item>
        /// </list>
        /// This technique should mean that it is never necessary to manually unregister a driver from ASCOM.
        /// </remarks>
        [ComUnregisterFunction]
        public static void UnregisterASCOM(Type t) {
            RegUnregASCOM(false);
        }

        #endregion

        /// <summary>
        /// Returns true if there is a valid connection to the driver hardware
        /// </summary>
        private bool IsConnected => connectedState;

        /// <summary>
        /// Use this function to throw an exception if we aren't connected to the hardware
        /// </summary>
        /// <param name="message"></param>
        private void CheckConnected(string message) {
            if (!IsConnected) {
                throw new ASCOM.NotConnectedException(message);
            }
        }

        /// <summary>
        /// Read the device configuration from the ASCOM Profile store
        /// </summary>
        internal void ReadProfile() {
            using (Profile driverProfile = new Profile()) {
                driverProfile.DeviceType = "ObservingConditions";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(driverID, traceStateProfileName, string.Empty, traceStateDefault));
                comPort = driverProfile.GetValue(driverID, comPortProfileName, string.Empty, comPortDefault);
                queryInterval = Convert.ToDouble(driverProfile.GetValue(driverID, queryIntervalProfileName, string.Empty, queryIntervalDefault));
            }
        }

        /// <summary>
        /// Write the device configuration to the  ASCOM  Profile store
        /// </summary>
        internal void WriteProfile() {
            using (Profile driverProfile = new Profile()) {
                driverProfile.DeviceType = "ObservingConditions";
                driverProfile.WriteValue(driverID, traceStateProfileName, tl.Enabled.ToString());
                driverProfile.WriteValue(driverID, comPortProfileName, comPort.ToString());
                driverProfile.WriteValue(driverID, queryIntervalProfileName, queryInterval.ToString());
            }
        }

        /// <summary>
        /// Log helper function that takes formatted strings and arguments
        /// </summary>
        /// <param name="identifier"></param>
        /// <param name="message"></param>
        /// <param name="args"></param>
        internal static void LogMessage(string identifier, string message, params object[] args) {
            var msg = string.Format(message, args);
            tl.LogMessage(identifier, msg);
        }

        /// <summary>
        /// Returns the difference between two DateTime objects in seconds
        /// </summary>
        /// <param name="time"></param>
        internal static double SecondsSince(DateTime time) {
            TimeSpan span = DateTime.Now - time;
            return span.Seconds;
        }
        #endregion
    }
}
