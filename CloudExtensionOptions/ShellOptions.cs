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
using System.Security.Claims;


namespace CloudExtensionOptions
{
    public partial class ShellOptions : Form
    {
        public ShellOptions()
        {
            InitializeComponent();

            String dataPath = GetFolderPath(SpecialFolder.LocalApplicationData);

            dataPath = Path.Combine(dataPath, "NCraft Software\\CloudShellExtension");
            m_settingsFilename = "settings.xml";
            m_settingsPathname = Path.Combine(dataPath, m_settingsFilename);

            this.ImageFormatComboBox.SelectedIndex = 0;
            this.ImageSizeComboBox.SelectedIndex = 0;

            m_xmlSettingsDoc = new XmlDocument();
            m_xmlSettingsDoc.Load(m_settingsPathname);

            XmlElement rootElem = m_xmlSettingsDoc.DocumentElement;

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
                        Process.Start("Regsvr32.exe", extensionDLLPathStr);
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
                        Process.Start("Regsvr32.exe", "/u " + extensionDLLPathStr);
                    }
                }
            }

        }

        private void PurchaseButton_Click(object sender, EventArgs e)
        {
            Process.Start("https://www.sourceforge.net");
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

        }

        private void OkButton_Click(object sender, EventArgs e)
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


            m_xmlSettingsDoc.Save(m_settingsPathname);
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
    }


}
