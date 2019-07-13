using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;
using ASCOM.Utilities;
using ASCOM.WeatherStick;

namespace ASCOM.WeatherStick {
    [ComVisible(false)]					// Form not registered for COM!
    public partial class SetupDialogForm : Form {
        public SetupDialogForm() {
            InitializeComponent();
            // Initialise current values of user settings from the ASCOM Profile
            InitUI();
        }

        private void cmdOK_Click(object sender, EventArgs e) // OK button event handler
        {
            // Place any validation constraint checks here
            // Update the state variables with results from the dialogue
            ObservingConditions.comPort = (string)comboBoxComPort.SelectedItem;
            ObservingConditions.tl.Enabled = chkTrace.Checked;
            ObservingConditions.queryInterval = Convert.ToDouble(textBoxQueryInterval.Text);
        }

        private void cmdCancel_Click(object sender, EventArgs e) // Cancel button event handler
        {
            Close();
        }

        private void BrowseToAscom(object sender, EventArgs e) // Click on ASCOM logo event handler
        {
            try {
                System.Diagnostics.Process.Start("http://ascom-standards.org/");
            } catch (System.ComponentModel.Win32Exception noBrowser) {
                if (noBrowser.ErrorCode == -2147467259)
                    MessageBox.Show(noBrowser.Message);
            } catch (System.Exception other) {
                MessageBox.Show(other.Message);
            }
        }

        private void InitUI() {
            chkTrace.Checked = ObservingConditions.tl.Enabled;

            // set the list of com ports to those that are currently available
            comboBoxComPort.Items.Clear();
            comboBoxComPort.Items.AddRange(System.IO.Ports.SerialPort.GetPortNames());

            // select the current port if possible
            if (comboBoxComPort.Items.Contains(ObservingConditions.comPort)) {
                comboBoxComPort.SelectedItem = ObservingConditions.comPort;
            }

            textBoxQueryInterval.Text = ObservingConditions.queryInterval.ToString();
        }
    }
}