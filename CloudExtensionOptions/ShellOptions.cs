using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Web;
using System.Xml;
using Microsoft.Win32;
using static System.Environment;
using System.Diagnostics;

using SKM.V3;
using SKM.V3.Methods;
using SKM.V3.Models;
using System.Xml.Linq;



namespace CloudExtensionOptions
{
    public partial class ShellOptions : Form
    {
        public ShellOptions()
        {
            InitializeComponent();

            StartPosition = FormStartPosition.CenterScreen;

            ImageFormatComboBox.SelectedIndex = 0;
            ImageSizeComboBox.SelectedIndex = 0;

            String dataPath = GetFolderPath(SpecialFolder.LocalApplicationData);

            dataPath = Path.Combine(dataPath, "NCraft Software\\CloudShellExtension");
            m_settingsFilename = "settings.xml";
            m_settingsPathname = Path.Combine(dataPath, m_settingsFilename);

            if (File.Exists(m_settingsPathname))
            {
                m_xmlSettingsDoc = new XmlDocument();

                if (m_xmlSettingsDoc != null)
                {
                    m_xmlSettingsDoc.Load(m_settingsPathname);

                    XmlElement rootElem = m_xmlSettingsDoc.DocumentElement;

                    if (rootElem != null)
                    {
                        XmlNode imageFormatElem = rootElem.SelectSingleNode("descendant::ImageFormat");
                        if (imageFormatElem != null && imageFormatElem.FirstChild != null)
                        {
                            m_imageFormat = imageFormatElem.FirstChild.Value;
                            if (m_imageFormat.Length > 0)
                            {
                                ImageFormatComboBox.Text = m_imageFormat;

                                int indexVal = ImageFormatComboBox.FindString(m_imageFormat);
                                if (indexVal > -1)
                                {
                                    ImageFormatComboBox.SelectedIndex = indexVal;
                                }
                            }
                        }

                        XmlNode imageSizeElem = rootElem.SelectSingleNode("descendant::ImageSize");
                        if (imageSizeElem != null && imageFormatElem.FirstChild != null)
                        {
                            m_imageSize = imageSizeElem.FirstChild.Value;
                            if (m_imageSize.Length > 0)
                            {
                                ImageSizeComboBox.Text = m_imageSize;
                                int indexVal = ImageSizeComboBox.FindString(m_imageSize);
                                if (indexVal > -1)
                                {
                                    ImageSizeComboBox.SelectedIndex = indexVal;
                                }
                            }
                        }

                        XmlNode keyElem = rootElem.SelectSingleNode("descendant::LicenseKey");
                        if (keyElem != null && keyElem.FirstChild != null)
                        {
                            m_keyValue = keyElem.FirstChild.Value;
                            if (m_imageSize.Length > 0)
                            {
                                LicenseKeyTextBox.Text = m_keyValue;
                            }
                        }
                    }
                }
            }

        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void EnableButton_Click(object sender, EventArgs e)
        {
            RegistryKey topCLSID = Registry.CurrentUser.OpenSubKey("SOFTWARE\\NCraft Software\\NCraft Point Cloud Shell Extension");

            if (topCLSID != null)
            {
                Object extensionDLLPath = topCLSID.GetValue("", "");
                if (extensionDLLPath != null)
                {
                    String extensionDLLPathStr = extensionDLLPath.ToString();

                    topCLSID.Close();

                    if (File.Exists(extensionDLLPathStr))
                    {
                        Process.Start("Regsvr32.exe", "\"" + extensionDLLPathStr + "\"");
                    }
                }
            }
        }

        private void DisableButton_Click(object sender, EventArgs e)
        {
            RegistryKey topCLSID = Registry.CurrentUser.OpenSubKey("SOFTWARE\\NCraft Software\\NCraft Point Cloud Shell Extension");

            if (topCLSID != null)
            {
                Object extensionDLLPath = topCLSID.GetValue("", "");
                if (extensionDLLPath != null)
                {
                    String extensionDLLPathStr = extensionDLLPath.ToString();

                    topCLSID.Close();

                    if (File.Exists(extensionDLLPathStr))
                    {
                        Process.Start("Regsvr32.exe", "/u " + "\"" + extensionDLLPathStr + "\"");
                    }
                }
            }

        }

        private void PurchaseButton_Click(object sender, EventArgs e)
        {

            //   Process.Start("https://www.sourceforge.net");

        }

        private void ImageFormatComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            m_imageFormat = ImageFormatComboBox.Items[ImageFormatComboBox.SelectedIndex].ToString();
        }

        private void ImageSizeComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            m_imageSize = ImageSizeComboBox.Items[ImageSizeComboBox.SelectedIndex].ToString();
        }

        private void LicenseKeyTextBox_TextChanged(object sender, EventArgs e)
        {
            m_keyValue = LicenseKeyTextBox.Text;
        }

        private void OkButton_Click(object sender, EventArgs e)
        {
            if (m_xmlSettingsDoc != null && m_xmlSettingsDoc.DocumentElement != null)
            {

                XmlElement rootElem = m_xmlSettingsDoc.DocumentElement;

                XmlNode imageFormatElem = rootElem.SelectSingleNode("descendant::ImageFormat");
                if (imageFormatElem != null && imageFormatElem.FirstChild != null)
                {
                    m_imageFormat = ImageFormatComboBox.Text;

                    if (m_imageFormat.Length > 0)
                    {
                        imageFormatElem.FirstChild.Value = m_imageFormat;
                    }
                }

                XmlNode imageSizeElem = rootElem.SelectSingleNode("descendant::ImageSize");
                if (imageSizeElem != null && imageSizeElem.FirstChild != null)
                {
                    m_imageFormat = ImageSizeComboBox.Text;

                    if (m_imageSize.Length > 0)
                    {
                        imageSizeElem.FirstChild.Value = m_imageSize;
                    }
                }

                XmlNode keyElem = rootElem.SelectSingleNode("descendant::LicenseKey");
                if (keyElem != null)
                {
                    if (m_isLicensed == true)
                    {
                        if (m_keyValue.Length > 0)
                        {
                            keyElem.InnerText = m_keyValue;
                        }
                    }
                    else
                    {
                        keyElem.InnerText = "";
                    }
                }

                m_xmlSettingsDoc.Save(m_settingsPathname);
            }


            this.Close();
        }

        private void CancelButton_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        XmlDocument m_xmlSettingsDoc;
        String m_settingsFilename;
        String m_settingsPathname;
        String m_keyValue;
        String m_imageFormat = "png";
        String m_imageSize = "1440x1024";
        bool m_isLicensed = false;

        private void ValidateButton_Click(object sender, EventArgs e)
        {
            // var licenseKey = "JJVRT-LBIZO-ZFNJN-LMYLV"; // <--  remember to change this to your license key
            var RSAPubKey = "<RSAKeyValue><Modulus>k8EvVN6X4dfoNdSXJNvwPYOM3qsldRYXBDRnWJH4ptmVgFX3unlQ53EdYu2cDxbSR9MMonpvmq2Dgs7WOM0Q4U8WVMNcexzxyTIoeImULzerPGQPN6UIZaGulUiPkFSSVXqD/cgYdlkzqro+x2hMExNMJB97ISsBy3aqYGdCtoUP8i0yjdbFcZKOns7ZaPYJE24MiYaIKp4dXNceWZ8w8SXWlxtvnqPe8KeXHRpf+/SrdoBd1IU6TXknLpL5gYpywfsQb8tk9rLtWFJLQEhkzID1KYwfSnBVXVn95u+OZI7h8CyQVl9sAmxAk0ETCQHE1hgesHwreSYA0X4KYBs0gw==</Modulus><Exponent>AQAB</Exponent></RSAKeyValue>";

            var auth = "WyI3MTg2OTc5OSIsInR4UkVKd2RhU2dUcG9vU091cHljNUZDWHY0TWltVmpDaFBERVFjdEwiXQ==";
            var result = Key.Activate(token: auth, parameters: new ActivateModel()
            {
                Key = m_keyValue,
                ProductId = 23571,  // <--  remember to change this to your Product Id
                Sign = true,
                MachineCode = Helpers.GetMachineCodePI(v: 2)
            });

            if (result == null || result.Result == ResultType.Error ||
                !result.LicenseKey.HasValidSignature(RSAPubKey).IsValid())
            {
                const string message = "Invalid License";
                const string caption = "Point Cloud Shell Extension";
                MessageBox.Show(message, caption, MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            else
            {
                // everything went fine if we are here!
                this.m_isLicensed = true;

                const string message = "License Validation Successful";
                const string caption = "Point Cloud Shell Extension";
                MessageBox.Show(message, caption, MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
        }
    }

}