////////////////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 Nathan C. Crews IV
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////////


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


namespace GLTFExtensionOptions
{
    public partial class ShellOptions : Form
    {
        public ShellOptions()
        {
            InitializeComponent();

            String dataPath = GetFolderPath(SpecialFolder.LocalApplicationData);

            dataPath = Path.Combine(dataPath, "NCrews Software\\GLTFShellExtension");
            m_settingsFilename = "settings.xml";
            m_settingsPathname = Path.Combine(dataPath, m_settingsFilename);

            if (File.Exists(m_settingsPathname) == false)
            {
                return;
            }

            this.ImageFormatComboBox.SelectedIndex = 0;
            this.ImageSizeComboBox.SelectedIndex = 0;

            m_xmlSettingsDoc = new XmlDocument();
            m_xmlSettingsDoc.Load(m_settingsPathname);

            if (m_xmlSettingsDoc.DocumentElement == null)
            {
                return;
            }

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
            RegistryKey topCLSID = Registry.CurrentUser.OpenSubKey("SOFTWARE\\NCrews Software\\NCrews GLTF Shell Extension");

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
            RegistryKey topCLSID = Registry.CurrentUser.OpenSubKey("SOFTWARE\\NCrews Software\\NCrews GLTF Shell Extension");

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
            Process.Start("https://www.buymeacoffee.com/nathancrews");
        }

        private void ImageFormatComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            m_imageFormat = ImageFormatComboBox.Items[ImageFormatComboBox.SelectedIndex].ToString();
        }

        private void ImageSizeComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            m_imageSize = ImageSizeComboBox.Items[ImageSizeComboBox.SelectedIndex].ToString();
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
        String m_imageFormat = "png";
        String m_imageSize = "1440x1024";

        private void label1_Click(object sender, EventArgs e)
        {
            Process.Start("https://www.buymeacoffee.com/nathancrews");
        }
    }


}
