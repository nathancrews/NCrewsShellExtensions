namespace GLTFExtensionOptions
{
    partial class ShellOptions
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.ImageFormatComboBox = new System.Windows.Forms.ComboBox();
            this.OkButton = new System.Windows.Forms.Button();
            this.CancelButton = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.ImageSizeComboBox = new System.Windows.Forms.ComboBox();
            this.EnableButton = new System.Windows.Forms.Button();
            this.DisableButton = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.PurchaseButton = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // ImageFormatComboBox
            // 
            this.ImageFormatComboBox.DisplayMember = "ImageFormat";
            this.ImageFormatComboBox.FormattingEnabled = true;
            this.ImageFormatComboBox.Items.AddRange(new object[] {
            "jpg",
            "png"});
            this.ImageFormatComboBox.Location = new System.Drawing.Point(103, 72);
            this.ImageFormatComboBox.Name = "ImageFormatComboBox";
            this.ImageFormatComboBox.Size = new System.Drawing.Size(121, 21);
            this.ImageFormatComboBox.TabIndex = 0;
            this.ImageFormatComboBox.ValueMember = "ImageFormat";
            this.ImageFormatComboBox.SelectedIndexChanged += new System.EventHandler(this.ImageFormatComboBox_SelectedIndexChanged);
            // 
            // OkButton
            // 
            this.OkButton.Location = new System.Drawing.Point(151, 151);
            this.OkButton.Name = "OkButton";
            this.OkButton.Size = new System.Drawing.Size(75, 23);
            this.OkButton.TabIndex = 1;
            this.OkButton.Text = "Ok";
            this.OkButton.UseVisualStyleBackColor = true;
            this.OkButton.Click += new System.EventHandler(this.OkButton_Click);
            // 
            // CancelButton
            // 
            this.CancelButton.Location = new System.Drawing.Point(264, 151);
            this.CancelButton.Name = "CancelButton";
            this.CancelButton.Size = new System.Drawing.Size(75, 23);
            this.CancelButton.TabIndex = 2;
            this.CancelButton.Text = "Cancel";
            this.CancelButton.UseVisualStyleBackColor = true;
            this.CancelButton.Click += new System.EventHandler(this.CancelButton_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(12, 75);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(71, 13);
            this.label2.TabIndex = 5;
            this.label2.Text = "Image Format";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(264, 77);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(59, 13);
            this.label4.TabIndex = 9;
            this.label4.Text = "Image Size";
            // 
            // ImageSizeComboBox
            // 
            this.ImageSizeComboBox.DisplayMember = "ImageSize";
            this.ImageSizeComboBox.FormattingEnabled = true;
            this.ImageSizeComboBox.Items.AddRange(new object[] {
            "1024x768",
            "1440x1024",
            "2048x1440"});
            this.ImageSizeComboBox.Location = new System.Drawing.Point(355, 74);
            this.ImageSizeComboBox.Name = "ImageSizeComboBox";
            this.ImageSizeComboBox.Size = new System.Drawing.Size(121, 21);
            this.ImageSizeComboBox.TabIndex = 8;
            this.ImageSizeComboBox.ValueMember = "ImageSize";
            this.ImageSizeComboBox.SelectedIndexChanged += new System.EventHandler(this.ImageSizeComboBox_SelectedIndexChanged);
            // 
            // EnableButton
            // 
            this.EnableButton.Location = new System.Drawing.Point(12, 23);
            this.EnableButton.Name = "EnableButton";
            this.EnableButton.Size = new System.Drawing.Size(227, 23);
            this.EnableButton.TabIndex = 11;
            this.EnableButton.Text = "Enable Shell Extension";
            this.EnableButton.UseVisualStyleBackColor = true;
            this.EnableButton.Click += new System.EventHandler(this.EnableButton_Click);
            // 
            // DisableButton
            // 
            this.DisableButton.Location = new System.Drawing.Point(264, 23);
            this.DisableButton.Name = "DisableButton";
            this.DisableButton.Size = new System.Drawing.Size(227, 23);
            this.DisableButton.TabIndex = 12;
            this.DisableButton.Text = "Disable Shell Extension";
            this.DisableButton.UseVisualStyleBackColor = true;
            this.DisableButton.Click += new System.EventHandler(this.DisableButton_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(164, 119);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(226, 13);
            this.label1.TabIndex = 15;
            this.label1.Text = "https://www.buymeacoffee.com/nathancrews";
            this.label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // PurchaseButton
            // 
            this.PurchaseButton.Location = new System.Drawing.Point(15, 114);
            this.PurchaseButton.Name = "PurchaseButton";
            this.PurchaseButton.Size = new System.Drawing.Size(120, 23);
            this.PurchaseButton.TabIndex = 14;
            this.PurchaseButton.Text = "Buy Me a Coffee?";
            this.PurchaseButton.UseVisualStyleBackColor = true;
            this.PurchaseButton.Click += new System.EventHandler(this.PurchaseButton_Click);
            // 
            // ShellOptions
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(506, 187);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.PurchaseButton);
            this.Controls.Add(this.DisableButton);
            this.Controls.Add(this.EnableButton);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.ImageSizeComboBox);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.CancelButton);
            this.Controls.Add(this.OkButton);
            this.Controls.Add(this.ImageFormatComboBox);
            this.Name = "ShellOptions";
            this.Text = "NCrews Shell Extension Options";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ComboBox ImageFormatComboBox;
        private System.Windows.Forms.Button OkButton;
        private System.Windows.Forms.Button CancelButton;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.ComboBox ImageSizeComboBox;
        private System.Windows.Forms.Button EnableButton;
        private System.Windows.Forms.Button DisableButton;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button PurchaseButton;
    }
}

