﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Threading;
using System.IO.Ports;
using System.IO;
using System.Timers;
using System.Collections;

namespace Motors
{
    public partial class Form1 : Form
    {
        SerialPort port = new SerialPort(); //обект класса SerialPort передачи данных через ком-порт
        
        
        //Thread th;  //объект класса Thread - поток
        byte[] Command = new byte[6];   //переменная для отправки команд
        
        public bool flag_connect = false;   //флаг соединения
        public bool flag_wait = false;   //флаг ожидания
        public bool flag_osi = false;   //флаг осей

        public string message1;

        public double[] gyro_x = new double[2];
        public double[] gyro_y = new double[2];
        public double[] gyro_z = new double[2];

        public double[] accel_x = new double[2];
        public double[] accel_y = new double[2];
        public double[] accel_z = new double[2];

        public double[] shag = new double[2];

        public double[] alfa_x = new double[2];
        public double[] alfa_y = new double[2];
        public double[] alfa_z = new double[2];

        public double[] v_x = new double[2];
        public double[] v_y = new double[2];

        public double[] s_x = new double[2];
        public double[] s_y = new double[2];

        public double time = 0.1;
        public int col = 1;
        public Graphics g1, g2, g3;

        public string[] m;
        ArrayList list = new ArrayList();
        
        public delegate void Write(string str, byte[] mas);
        public Write mydel;

        //private void WriteMethod(string indata);

        public Form1()
        {
            InitializeComponent();

            comboBox1.Items.AddRange(SerialPort.GetPortNames());    //создаем список доступных портов в комбобокс1

            if (SerialPort.GetPortNames().Length != 0)  //если найден хоть один
            {
                comboBox1.Text = SerialPort.GetPortNames()[0];  //выводим первый доступный
            }

            System.Windows.Forms.Control.CheckForIllegalCrossThreadCalls = false;   //без этого не работает

            mydel = new Write(WriteMethod);
            g1 = panel1.CreateGraphics();    //рисуем график на панели 1
            g1.Clear(Color.White);

            g2 = panel2.CreateGraphics();    //рисуем график на панели 1
            g2.Clear(Color.White);

            g3 = panel3.CreateGraphics();    //рисуем график на панели 1
            g3.Clear(Color.White);

            //for (int i = 0; i < 1000; i++)
            //{
            //    shag[i] = i;
            //}
        }

        //////////////////////////////////////////////////////////////////
        // Кнопка обновить
        //////////////////////////////////////////////////////////////////

        private void button9_Click(object sender, EventArgs e)
        {
            comboBox1.Items.Clear();    //удаляем все что есть

            comboBox1.Items.AddRange(SerialPort.GetPortNames());    //добавляем в список доступные порты
        }

        //////////////////////////////////////////////////////////////////
        // Кнопка Подлючить/Отключить
        //////////////////////////////////////////////////////////////////
        private void button10_Click(object sender, EventArgs e)
        {
            if (!flag_connect)  //если соединение не установлено
            {
                textBox21.Text = "Connecting...";
                textBox21.Update();

                port.PortName = comboBox1.Text; //имя порта из списка1
                port.BaudRate = int.Parse(comboBox2.Text);  //скорость из списка 2
                port.DataReceived += new SerialDataReceivedEventHandler(DataReceviedHandler);

                // Set the read/write timeouts
                port.ReadTimeout = 500;
                port.WriteTimeout = 500;

                try
                {
                    port.Open();    //устанавливаем соединение
                }
                catch
                {
                    textBox21.Text = "Error";
                    return;
                }
                button10.Text = "Disconnect";    
                flag_connect = true;    //устанавливем флаг соединения 1
                textBox21.Text = "OK";

                g1.Clear(Color.White);
                g2.Clear(Color.White);
                g3.Clear(Color.White);

                return; //выходим из функции

            }

            if (flag_connect)   //если соединение установлено
            {
                flag_connect = false;   //сбрасываем флаг
            
                port.Close();   //закрываем соединение
                button10.Text = "Connect";   //меняем имя кнопки
                textBox21.Text = "Not connect";

                //g = panel1.CreateGraphics();    //рисуем график на панели 1
                //g.Clear(Color.White);
                flag_osi = false;

                //Grafik gr = new Grafik(shag, x, g, panel1.ClientSize, Color.Red,30,5); //обьект класса для первого графика
                //Grafik gr1 = new Grafik(shag, y, g, panel1.ClientSize, Color.Green, 30,5);  //для второго графика
                //gr.Osi();
                //gr.ris();
                //gr1.ris();

                col = 0;
                gyro_x[0] = gyro_x[1] = gyro_y[0] = gyro_y[1] = shag[0] = shag[1] = 0;
                gyro_z[0] = gyro_z[1] = alfa_x[0] = alfa_x[1] = 0;
                accel_x[0] = accel_x[1] = accel_y[0] = accel_y[1] = accel_z[0] = accel_z[1] = 0;

                return;
            }
        }

        public void SendRF(byte[] command)
        {
                           
            //передача команды

            if (port.IsOpen)
            {
                port.Write(command, 0, command.Length);
                //flag_wait = true;
                Thread.Sleep(200);
            }
            else
            {
                textBox21.Text = "Not connect";
            }
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка вперед 1 мотор
        ////////////////////////////////////////////////////////////////////////
        private void button1_Click(object sender, EventArgs e)
        {
            textBox3.Text = "Forward 1";   //выводим в текстбокс

            Command[0] = 125;
            Command[1] = 1;
            Command[2] = Convert.ToByte(Convert.ToInt16(textBox4.Text) / 10);
            
            Command[5] = 1;
            SendRF(Command);
            
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка назад 1 мотор
        ////////////////////////////////////////////////////////////////////////
        private void button2_Click(object sender, EventArgs e)
        {
            textBox3.Text = "Back 1";

            Command[0] = 125;
            Command[1] = 2;
            Command[2] = Convert.ToByte(Convert.ToInt16(textBox4.Text) / 10);

            Command[5] = 1;
            SendRF(Command);
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка Стоп 1 мотор
        ////////////////////////////////////////////////////////////////////////
        private void button4_Click(object sender, EventArgs e)
        {
            textBox3.Text = "STOP 1";

            Command[0] = 125;
            Command[1] = 0;
            Command[2] = 0;

            Command[5] = 1;
            SendRF(Command);
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка вперед 2 мотор
        ////////////////////////////////////////////////////////////////////////
        private void button3_Click(object sender, EventArgs e)
        {
            textBox18.Text = "Forward 2";   //выводим в текстбокс

            Command[0] = 125;
            Command[3] = 1;
            Command[4] = Convert.ToByte(Convert.ToInt16(textBox5.Text) / 10);

            Command[5] = 1;
            SendRF(Command);
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка назад 2 мотор
        ////////////////////////////////////////////////////////////////////////
        private void button7_Click(object sender, EventArgs e)
        {
            textBox18.Text = "Back 2";

            Command[0] = 125;
            Command[3] = 2;
            Command[4] = Convert.ToByte(Convert.ToInt16(textBox5.Text) / 10);

            Command[5] = 1;
            SendRF(Command);
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка Стоп 2 мотор
        ////////////////////////////////////////////////////////////////////////
        private void button12_Click(object sender, EventArgs e)
        {
            textBox18.Text = "STOP 2";

            Command[0] = 125;
            Command[3] = 0;
            Command[4] = 0;

            Command[5] = 1;
            SendRF(Command);
        }
                

        ////////////////////////////////////////////////////////////////////////
        //кнопка вперед 1 и 2 моторы
        ////////////////////////////////////////////////////////////////////////
        private void button6_Click(object sender, EventArgs e)
        {
            textBox3.Text = "Forward 1";
            textBox18.Text = "Forward 2";

            Command[0] = 125;

            if (checkBox1.Checked == false)
            {
                Command[1] = 1;                                 //direction
            }
            else
            {
                Command[1] = 0;                                 //direction
            }
            Command[2] = Convert.ToByte(Convert.ToInt16(textBox4.Text) / 10);      //speed high


            if (checkBox2.Checked == false)
            {
                Command[3] = 1;                                 //direction
            }
            else
            {
                Command[3] = 0;                                 //direction
            }
            Command[4] = Convert.ToByte(Convert.ToInt16(textBox5.Text) / 10);     //speed high

            Command[5] = 1;                                 //enable

            SendRF(Command);
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка назад 1 и 2 моторы
        ////////////////////////////////////////////////////////////////////////
        private void button8_Click(object sender, EventArgs e)
        {
            textBox3.Text = "Back 1";
            textBox18.Text = "Back 2";

            Command[0] = 125;

            if (checkBox1.Checked == false)
            {
                Command[1] = 0;                                 //direction
            }
            else
            {
                Command[1] = 1;                                 //direction
            }
            Command[2] = Convert.ToByte(Convert.ToInt16(textBox4.Text) / 10);     //speed high


            if (checkBox2.Checked == false)
            {
                Command[3] = 0;                                 //direction
            }
            else
            {
                Command[3] = 1;                                 //direction
            }
            Command[4] = Convert.ToByte(Convert.ToInt16(textBox5.Text) / 10);     //speed high

            Command[5] = 1;                                 //enable

            SendRF(Command);
        }

        ////////////////////////////////////////////////////////////////////////
        //кнопка стоп 1 и 2 моторы
        ////////////////////////////////////////////////////////////////////////
        private void button16_Click(object sender, EventArgs e)
        {
            textBox3.Text = "STOP 1";
            textBox18.Text = "STOP 2";

            Command[0] = 125;

            Command[1] = 0;
            Command[2] = 0;

            Command[3] = 0;
            Command[4] = 0;

            Command[5] = 0;

            SendRF(Command);
        }

        ////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////


        //событие при закрытии окна
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {

            if (flag_connect)   //если соединение установлено
            {
                flag_connect = false;   //сбрасываем флаг

                
                //th.Join();  //ждем завершения потока
                port.Close();   //закрываем соединение

            }
        }



        //////////////////////////////////////////////////////////////////
        // Метод записи принятых данных в текстбокс
        //////////////////////////////////////////////////////////////////
        private void WriteMethod(string indata, byte[] mas)
        {
            double kk=0.2;
            double accel_ugol_x = 0;
            double accel_ugol_y = 0;
            double accel_ugol_z = 0;
            int aa = 0;
            
            double ang = 0;

            //ASCII
            try
            {
                if (indata.Length < 10)
                {
                    return;
                }
                textBox1.AppendText(indata);
                indata = indata.Replace("\r\n", null);
                indata = indata.Replace('.', ',');

                list.Add(indata);

                if (indata.IndexOf("-", 0, 1) != -1)
                {
                    indata = indata.Replace("-", null);
                    ang = -1;
                }
                else
                {
                    ang = 1;
                }

                
                try
                {

                    ang = ang * Convert.ToDouble(indata);
                    alfa_x[1] = ang;
                    shag[1] = col;
                    col++;

                    double m_x = 2.0, m_y = 0.05;

                    Grafik gr = new Grafik(shag, alfa_x, g1, panel1.ClientSize, Color.Red, m_x, m_y); //обьект класса для первого графика
                    if (!flag_osi)
                    {
                        gr.Osi();
                        flag_osi = true;
                    }

                    gr.ris();

                    shag[0] = shag[1];
                    alfa_x[0] = alfa_x[1];
                    
                    if (col == (int)(panel1.Width / gr.mas_x))
                    {
                        button5.PerformClick();

                    }
                }
                catch { }

            }
            catch { }

            //DEC
            //for (int i = 1; i < mas.Length; i++)
            //{
            //    textBox1.AppendText(Convert.ToString((sbyte)mas[i]));
            //    textBox1.AppendText(" ");
            //}

            try
            {
                if (mas[0] == 10)
                {
                    gyro_x[1] = (sbyte)(mas[1]);
                    gyro_y[1] = (sbyte)(mas[2]);
                    gyro_z[1] = (sbyte)(mas[3]);
                    accel_x[1] = (sbyte)(mas[4]);
                    accel_y[1] = (sbyte)(mas[5]);
                    accel_z[1] = (sbyte)(mas[6]);
                    shag[1] = col;

                    textBox2.Text = gyro_x[1].ToString();
                    textBox6.Text = gyro_y[1].ToString();
                    textBox7.Text = gyro_z[1].ToString();

                    textBox8.Text = accel_x[1].ToString();
                    textBox9.Text = accel_y[1].ToString();
                    textBox10.Text = accel_z[1].ToString();

                    col++;

                    accel_ugol_x = Math.Atan2(accel_y[1], accel_z[1]) * (180 / Math.PI);
                    accel_ugol_y = Math.Atan2(accel_x[1], accel_z[1]) * (180 / Math.PI);
                    accel_ugol_z = Math.Atan2(accel_x[1], accel_y[1]) * (180 / Math.PI);

                    //if (accel_ugol_x < 0)
                    //{
                    //    accel_ugol_x = 180 + (180 - Math.Abs(accel_ugol_x));
                    //}
                    //if (accel_ugol_y < 0)
                    //{
                    //    accel_ugol_y = 180 + (180 - Math.Abs(accel_ugol_y));
                    //}
                    

                    
                    textBox11.Text = (accel_ugol_x).ToString();
                    textBox14.Text = (accel_ugol_y).ToString();
                    textBox16.Text = (accel_ugol_z).ToString();

                    alfa_x[1] = (1 - kk) * (alfa_x[0] + gyro_x[1] * time) + kk * (accel_ugol_x / 5.625);
                    alfa_y[1] = (1 - kk) * (alfa_y[0] + gyro_y[1] * time) + kk * (accel_ugol_y / 5.625);
                    //alfa_z[1] = (alfa_z[0] + gyro_z[1] * time);
                    alfa_z[1] = (1 - kk) * (alfa_z[0] + gyro_z[1] * time) + kk * (accel_ugol_z / 5.625);

                    textBox12.Text = (alfa_x[1] * 5.625).ToString();
                    textBox13.Text = (alfa_y[1] * 5.625).ToString();
                    textBox15.Text = (alfa_z[1] * 5.625).ToString();

                    //alfa_x[1] = (alfa_x[0] + gyro_x[0] * time); 
                    //v_x[1] = x[1] * time + v_x[0];
                    //v_y[1] = y[1] * time + v_y[0];

                    //s_x[1] = s_x[0] + v_x[1] * time + time * time * 0.5 * x[1];
                    //s_y[1] = s_y[0] + v_y[1] * time + time * time * 0.5 * y[1];

                    //g = panel1.CreateGraphics();    //рисуем график на панели 1
                    //g.Clear(Color.White);

                    double m_x = 2.0, m_y = 2.0;

                    Grafik gr = new Grafik(shag, gyro_x, g1, panel1.ClientSize, Color.Red, m_x, m_y); //обьект класса для первого графика
                    Grafik gr1 = new Grafik(shag, gyro_y, g1, panel1.ClientSize, Color.Green, m_x, m_y);  //для второго графика
                    Grafik gr2 = new Grafik(shag, gyro_z, g1, panel1.ClientSize, Color.Blue, m_x, m_y); 

                    Grafik v_gr = new Grafik(shag, alfa_x, g2, panel1.ClientSize, Color.Red, m_x, m_y); //обьект класса для первого графика
                    Grafik v_gr1 = new Grafik(shag, alfa_y, g2, panel1.ClientSize, Color.Green, m_x, m_y);  //для второго графика
                    Grafik v_gr2 = new Grafik(shag, alfa_z, g2, panel1.ClientSize, Color.Blue, m_x, m_y); 

                    //Grafik s_gr = new Grafik(shag, s_x, g3, panel1.ClientSize, Color.Red, m_x, m_y); //обьект класса для первого графика
                    //Grafik s_gr1 = new Grafik(shag, s_y, g3, panel1.ClientSize, Color.Green, m_x, m_y);  //для второго графика

                    if (!flag_osi)
                    {
                        gr.Osi();
                        v_gr.Osi();
                        //s_gr.Osi();
                        flag_osi = true;
                    }
                    gr.ris();
                    gr1.ris();
                    gr2.ris();

                    v_gr.ris();
                    v_gr1.ris();
                    v_gr2.ris();

                    //s_gr.ris();
                    //s_gr1.ris();

                    gyro_x[0] = gyro_x[1];
                    gyro_y[0] = gyro_y[1];
                    gyro_z[0] = gyro_z[1];

                    accel_x[0] = accel_x[1];
                    accel_y[0] = accel_y[1];
                    accel_z[0] = accel_z[1];

                    shag[0] = shag[1];

                    alfa_x[0] = alfa_x[1];
                    alfa_y[0] = alfa_y[1];
                    alfa_z[0] = alfa_z[1];
                    //v_x[0] = v_x[1];
                    //v_y[0] = v_y[1];

                    //s_x[0] = s_x[1];
                    //s_y[0] = s_y[1];

                    
                    if (col == (int)(panel1.Width/gr.mas_x))
                    {
                        button5.PerformClick();

                    }
                }
            }
            catch { }
            
        }

        //////////////////////////////////////////////////////////////////
        // Событие приема данных
        //////////////////////////////////////////////////////////////////
        private void DataReceviedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            string indata = "";
            byte[] rec;

            Thread.Sleep(10);

            if (sp.IsOpen)
            {
                rec = new byte[sp.BytesToRead];
            }
            else rec = new byte[1];


            //ASCII
            try
            {
                indata = sp.ReadExisting();
            }
            catch { }


            //DEC
            //try
            //{
            //    sp.Read(rec, 0, sp.BytesToRead);
            //}
            //catch { }

            mydel(indata, rec);
        }

        //////////////////////////////////////////////////////////////////
        // Кнопка выход
        //////////////////////////////////////////////////////////////////

        private void button11_Click(object sender, EventArgs e)
        {
            if (port.IsOpen)    //если установлено соединение
            {
                flag_connect = false;   //сбрасываем флаг

                                
                //th.Join();  //ждем завершение потока
                port.Close();   //закрываем соединение

            }
            Application.Exit(); //выходим из программы
        }

        //////////////////////////////////////////////////////////////////
        // Событие при нажатии на кнопку
        //////////////////////////////////////////////////////////////////
        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Q)
            {
                button1.PerformClick();
            }

            if (e.KeyCode == Keys.W)
            {
                button4.PerformClick();
            }

            if (e.KeyCode == Keys.E)
            {
                button2.PerformClick();
            }

            if (e.KeyCode == Keys.A)
            {
                button3.PerformClick();
            }
            
            if (e.KeyCode == Keys.S)
            {
                button12.PerformClick();
            }

            if (e.KeyCode == Keys.D)
            {
                button7.PerformClick();
            }

            if (e.KeyCode == Keys.Z)
            {
                button6.PerformClick();
            }

            if (e.KeyCode == Keys.X)
            {
                button16.PerformClick();
            }

            if (e.KeyCode == Keys.C)
            {
                button8.PerformClick();
            }

        }

        private void button5_Click(object sender, EventArgs e)
        {
            textBox1.Clear();

            flag_osi = false;

            col = 0;
            //gyro_x[0] = gyro_x[1] = gyro_y[0] = gyro_y[1] = 0;
            shag[0] = shag[1] = 0;
            //gyro_z[0] = gyro_z[1] = 0;
            //accel_x[0] = accel_x[1] = accel_y[0] = accel_y[1] = accel_z[0] = accel_z[1] = 0;
            //alfa_x[0] = alfa_x[1] = 0;
            //x[0] = x[1] = y[0] = y[1] = shag[0] = shag[1] = 0;

            try
            {
                g1.Clear(Color.White);
                g2.Clear(Color.White);
                g3.Clear(Color.White);
            }
            catch { }
        }

        private void Form1_KeyPress(object sender, KeyPressEventArgs e)
        {
            //e.Handled = true;
            if (e.KeyChar == (char)Keys.Q)
            {
                button1.PerformClick();
            }

            if (e.KeyChar == (char)Keys.W)
            {
                button4.PerformClick();
            }

            if (e.KeyChar == (char)Keys.E)
            {
                button2.PerformClick();
            }

            if (e.KeyChar == (char)Keys.A)
            {
                button3.PerformClick();
            }
            
            if (e.KeyChar == (char)Keys.S)
            {
                button12.PerformClick();
            }

            if (e.KeyChar == (char)Keys.D)
            {
                button7.PerformClick();
            }

            if (e.KeyChar == (char)Keys.Z)
            {
                button6.PerformClick();
            }

            if (e.KeyChar == (char)Keys.X)
            {
                button16.PerformClick();
            }

            if (e.KeyChar == (char)Keys.C)
            {
                button8.PerformClick();
            }

            
            Thread.Sleep(100);
        }

        private void button17_Click(object sender, EventArgs e)
        {
            textBox4.Text = textBox5.Text = textBox20.Text;
        }

        private void button18_Click(object sender, EventArgs e)
        {
            byte kp, kd, ki;

            kp = Convert.ToByte(double.Parse(textBox22.Text) * 100);
            kd = Convert.ToByte(double.Parse(textBox19.Text) * 1000);
            ki = Convert.ToByte(double.Parse(textBox17.Text) * 1000000);

            if (kp < 0 || kd < 0 || ki < 0)
            {
                MessageBox.Show("Коэффициенты ниже минимального значения");
                return;
            }

            if (kp > 255 || kd > 255 || ki > 255)
            {
                MessageBox.Show("Коэффициенты выше максимального значения");
                return;
            }
                        

            byte[] command = new byte[6];   //переменная для отправки команд

            command[0] = 150;
            command[1] = kp;
            command[2] = kd;
            command[3] = ki;
            command[4] = 0;
            command[5] = 0;
            
            SendRF(command);
        }

        private void button19_Click(object sender, EventArgs e)
        {
            byte[] command = new byte[6];   //переменная для отправки команд

            for (int i = 0; i < 6; i++)
            {
                command[i] = 0;
            }

            command[0] = 151;
            
            SendRF(command);
        }

        private void button20_Click(object sender, EventArgs e)
        {
            int kp = 10, kd = 10, ki = 10;

            if (port.IsOpen)
            {
                textBox4.Text = "50";
                textBox5.Text = "50";

                textBox4.Update();
                textBox5.Update();
                
                for (kd = 10; kd <= 150; kd += 30)
                {
                    for (kp = 10; kp <= 150; kp += 30)
                    {
                        //for (ki = 10; ki <= 150; ki += 30)
                        //{
                        textBox22.Text = kp.ToString();
                        textBox19.Text = kd.ToString();
                        textBox17.Text = ki.ToString();

                        textBox22.Update();
                        textBox19.Update();
                        textBox17.Update();

                        button18.PerformClick();
                        Thread.Sleep(100);

                        button19.PerformClick();
                        Thread.Sleep(100);

                        button1.PerformClick();
                        Thread.Sleep(5500);

                        button4.PerformClick();
                        Thread.Sleep(700);

                        button4.PerformClick();
                        Thread.Sleep(700);

                        //button2.PerformClick();
                        //Thread.Sleep(700);
                        // }

                    }
                }

                StreamWriter sw = new StreamWriter("1.txt", false);

                sw.Write(textBox1.Text);

                sw.Close();
            }
            else
            {
                textBox21.Text = "Not connect";
            }
        }

        private void button13_Click(object sender, EventArgs e)
        {
            byte k_theta, k_phi, linear_speed;
            
            try
            {
                k_theta = Convert.ToByte(double.Parse(textBox23.Text) * 10);
                k_phi = Convert.ToByte(double.Parse(textBox24.Text) * 100);
                linear_speed = Convert.ToByte(double.Parse(textBox25.Text) / 1);
            }
            catch
            {
                MessageBox.Show("Коэффициенты имеют неверное значение");
                return;
            }
            


            byte[] command = new byte[6];   //переменная для отправки команд

            command[0] = 160;
            command[1] = k_theta;
            command[2] = k_phi;
            command[3] = linear_speed;
            command[4] = 0;
            command[5] = 0;

            SendRF(command);
        }

        private void button14_Click(object sender, EventArgs e)
        {
            byte[] command = new byte[6];   //переменная для отправки команд

            for (int i = 0; i < 6; i++)
            {
                command[i] = 0;
            }

            command[0] = 161;

            SendRF(command);
        }

        private void button15_Click(object sender, EventArgs e)
        {
            byte[] command = new byte[6];   //переменная для отправки команд

            for (int i = 0; i < 6; i++)
            {
                command[i] = 0;
            }

            command[0] = 170;

            SendRF(command);
        }

        private void button21_Click(object sender, EventArgs e)
        {
            byte[] command = new byte[6];   //переменная для отправки команд

            byte coeff;

            try
            {
                coeff = Convert.ToByte(double.Parse(textBox26.Text) * 100);
                
            }
            catch
            {
                MessageBox.Show("Коэффициенты имеют неверное значение");
                return;
            }

            for (int i = 0; i < 6; i++)
            {
                command[i] = 0;
            }

            command[0] = 180;
            command[1] = coeff;

            SendRF(command);
        }

        private void button22_Click(object sender, EventArgs e)
        {
            byte[] command = new byte[6];   //переменная для отправки команд

            byte t;

            try
            {
                t = Convert.ToByte(double.Parse(textBox27.Text) );

            }
            catch
            {
                MessageBox.Show("Коэффициенты имеют неверное значение");
                return;
            }

            for (int i = 0; i < 6; i++)
            {
                command[i] = 0;
            }

            command[0] = 190;
            command[1] = t;

            SendRF(command);
        }

        
    }

    class Grafik
    {
        public double[] x, y;
        public double mas_x, mas_y;  //Масштаб
        public Graphics graph;
        public Pen myPen;
        public Pen myPen2 = new Pen(System.Drawing.Color.LightBlue);
        public Pen myPen3 = new Pen(System.Drawing.Color.Chocolate);
        public Size size;
        public int x_null = 10, y_null = 115;


        public Grafik(double[] x, double[] y, Graphics graph, Size size, Color col, double mas_x, double mas_y)
        {
            this.x = new double[x.Length];
            this.y = new double[y.Length];
            this.x = x;
            this.y = y;
            this.graph = graph;
            this.myPen = new Pen(col);
            this.size = size;
            this.mas_x = mas_x;
            this.mas_y = mas_y;

        }

        private float m_x(double a)
        {
            float b;
            b = Convert.ToInt32(a * mas_x + x_null);
            return b;
        }

        private float m_y(double a)
        {
            float b;
            b = Convert.ToInt32(a * (-1) * mas_y + y_null);
            return b;
        }
        public void ris()
        {
            for (int i = 0; i < x.Length - 1; i++)
            {
                graph.DrawLine(myPen, m_x(x[i]), m_y(y[i]), m_x(x[i + 1]), m_y(y[i + 1]));
            }
        }

        private void delenie(Pen myPen, double x1, double x2, double y1, double y2)
        {
            x1 = x1 * mas_x + size.Width / 2;
            x2 = x2 * mas_x + size.Width / 2;
            y1 = y1 * mas_y + size.Height / 2;
            y2 = y2 * mas_y + size.Height / 2;
            graph.DrawLine(myPen, Convert.ToInt32(x1), Convert.ToInt32(y1), Convert.ToInt32(x2), Convert.ToInt32(y2));
        }
        public void Setka()
        {
            for (double xx = -size.Width / 2 * mas_x; xx < size.Width / 2 * mas_x; xx++)
            {
                delenie(myPen2, xx, xx, size.Height / mas_x, -size.Height / mas_x);
            }
            for (double yy = -size.Height / 2 * mas_y; yy < size.Height / 2 * mas_y; yy++)
            {
                delenie(myPen2, size.Width / mas_y, -size.Width / mas_y, yy, yy);
            }

            graph.DrawLine(new Pen(Color.Black), 0, size.Height / 2, size.Width, size.Height / 2);
            graph.DrawLine(new Pen(Color.Black), size.Width / 2, 0, size.Width / 2, size.Height);
        }
        public void Osi()
        {
            graph.DrawLine(new Pen(Color.LightBlue, 2), 0, y_null, size.Width, y_null);
            graph.DrawLine(new Pen(Color.LightBlue, 2), x_null, 0, x_null, size.Height);
        }
       
    }

}
