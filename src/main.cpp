/*
 * Project: ESP32 Low-Frequency Spectrum Analyzer
 * Description: A spectrum analyzer project for the ESP32 microcontroller that analyzes low-frequency signals.
 * Author: Joseph Mesches
 * University: Colorado State University
 * Course: ECE202
 * Date: Spring 2024
 *
 * Copyright (c) [Year]
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Library Includes
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>
#include <arduinoFFT.h>
#include <Free_Fonts.h>
#include <test_data_1.h> //EKG Test Data
#include <test_data_2.h> //Sine Wave Test Data

//Standard Definitions
#define BAUD_RATE 115200
#define WELCOME_TIME 1000

// Pin Definitions
/* ACQUISITION INTERFACES*/
#define SIGNAL_PIN 34
/* BUTTONS */
#define BUTTON_01 13
#define BUTTON_02 12
//#define BUTTON_03 14

/* Constant Definitions */
//Buttons
#define DEBOUNCE_DELAY 250 //250ms debounce delay
//MEASUREMENT
#define DEFAULT_SAMPLE_FREQ 1000
#define DEFAULT_BUFFER_SIZE 2048
//Graphing
#define FFT_GRID_COLOR TFT_BLUE
#define FFT_TRACE_COLOR TFT_GREEN
#define TIME_GRID_COLOR TFT_BLUE
#define TIME_ZERO_COLOR TFT_WHITE
#define TIME_TRACE_COLOR TFT_RED
#define DEFAULT_TIME_Y_MIN -4
#define DEFAULT_TIME_Y_MAX 4
#define DEFAULT_TIME_Y_INC 1
//Tool bar
#define TOOLBAR_REFRESH_PERIOD 50
#define TOOLBAR_TEXT_COLOR TFT_RED
#define TOOLBAR_TEXT_BG_COLOR TFT_BLUE
#define TOOLBAR_BG_COLOR TFT_BLUE

/* Instantiate Variables */
//Timing
volatile unsigned long sample_start_time, sample_end_time;
//Buttons
volatile unsigned long button_01_last_millis = 0;
volatile unsigned long button_02_last_millis = 0;
//volatile unsigned long button_03_last_millis = 0;
volatile bool button_01_pressed = false;
volatile bool button_02_pressed = false;
//volatile bool button_03_pressed = false;
unsigned int display_mode = 0; //0 - Graph, 1 - Data
unsigned int data_mode = 1; //0 - Hall Sensor, 1 - Analog, 2 - Test 1, 3 - Test 2
volatile bool acquire_data = false;
/* TEST DATA */
unsigned int data_index_set1 = 0;
unsigned int last_data_point_set1 = 0;
unsigned int data_index_set2 = 0;
unsigned int last_data_point_set2 = 0;
float current_data_point = 0.00;
//SINE WAVE TEST DATA PARAMETERS
const unsigned int DATA_FREQ_set1 = 1000;
const unsigned int DATA_PERIOD_set1 = 1E6 / DATA_FREQ_set1;
//ELECTROCARDIOGRAM TEST DATA PARAMETERS
const unsigned int DATA_FREQ_set2 = 200;
const unsigned int DATA_PERIOD_set2 = 1E6 / DATA_FREQ_set2;
//Buffer Parameters
volatile unsigned long last_acquisition = 0;
unsigned int SAMPLE_FREQ = DEFAULT_SAMPLE_FREQ;
unsigned int SAMPLE_PERIOD;
const unsigned int BUFFER_SIZE = DEFAULT_BUFFER_SIZE;
const unsigned int BUFFER_POWER = log2(BUFFER_SIZE);
const unsigned int SEC_TO_GRAPH = 10;
//Buffers
float DATA_BUFFER[BUFFER_SIZE];
float COMPLEX_BUFFER[BUFFER_SIZE];
float TIME_BUFFER[BUFFER_SIZE];
volatile unsigned int buffer_index = 0;
//Screen Properties
unsigned long last_toolbar_refresh = 0;
char toolbar_left[10] = "LEFT";
char toolbar_center[10] = "CENTER";
char toolbar_right[10] = "RIGHT";
unsigned long last_refresh = 0;
unsigned int current_mode = display_mode;
bool screen_initialized = false;
//Time Series
float timeseries_x_min = 0;
float timeseries_x_max = BUFFER_SIZE;
float timeseries_x_inc = timeseries_x_max / 10;
float timeseries_y_min = DEFAULT_TIME_Y_MIN;
float timeseries_y_max = DEFAULT_TIME_Y_MAX;
float timeseries_y_inc = DEFAULT_TIME_Y_INC;
//Frequency Domain
float frequency_x_min = 0;
float frequency_x_max = BUFFER_SIZE / 2;
float frequency_x_inc = BUFFER_SIZE / 20;
float frequency_y_min = 0;
float frequency_y_max = 4;
float frequency_y_inc = 1;
float frequency_magnitude_max = 4;

/* CREATE OBJECTS */
//Screen Object
TFT_eSPI tft = TFT_eSPI();
//Graph Objects
GraphWidget timeseries_graph = GraphWidget(&tft);
TraceWidget timeseries_trace = TraceWidget(&timeseries_graph);
GraphWidget frequency_graph = GraphWidget(&tft);
TraceWidget frequency_trace = TraceWidget(&frequency_graph);
//FFT Object
ArduinoFFT<float> FFT = ArduinoFFT<float>(DATA_BUFFER, COMPLEX_BUFFER, BUFFER_SIZE, SAMPLE_FREQ);

/* Function Declarations */
/* BUTTON LOGIC*/
//Button Debounce
void IRAM_ATTR buttonDebounce01();
void IRAM_ATTR buttonDebounce02();
//void IRAM_ATTR buttonDebounce03();
//Button Function
//void ChangeDisplayMode();
void ChangeDataMode();
void ChangeAcquisitionMode();
/* TFT SCREEN LOGIC*/
//Define Screens
void DrawGraphScreen();
void DrawDataScreen();
//Welcome Screen
void WriteWelcomeScreen();
//Graph Screen
void DrawFrequencyGraph();
void DrawTimeGraph();
void ScaleTimeGraph();
void ScaleFrequencyGraph();
void PlotFrequencyGraph();
void PlotTimeGraph();
//Data Screen
void WriteDataScreen();
//Tool bar
void DrawToolBar();
/* DATA ACQUISITION LOGIC*/
void AcquireData();
float AcquireAnalog(unsigned int pin = SIGNAL_PIN);
float AcquireTest(unsigned int set);
float AcquireHall();
/* BUFFER LOGIC*/
void ResetBuffers();
void WriteBuffer(float data);
/* FFT LOGIC*/
void RunFFT();
/* LED LOGIC*/
void TurnOffLED();
void SetLEDColor(int color);

void setup() {
  //Initialize Serial Communication
  while (!Serial) {
  Serial.begin(BAUD_RATE);
  }
  Serial.printf("Serial Communication Established. Baud Rate: %d\n", BAUD_RATE);

  //Set PinModes
  pinMode(BUTTON_01, INPUT_PULLUP);
  pinMode(BUTTON_02, INPUT_PULLUP);
  //pinMode(BUTTON_03, INPUT_PULLUP);

  //Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(BUTTON_01), buttonDebounce01, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_02), buttonDebounce02, FALLING);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_03), buttonDebounce03, FALLING);

  //Initiate TFT Screen
  tft.begin();
  tft.setRotation(1);
  Serial.printf("TFT Initialized. Width: %d. Height: %d.\n", tft.width(), tft.height());
  //Write Welcome Screen (Inherent Delay of WELCOME_TIME)
  WriteWelcomeScreen();
}

void loop() {
  //Button Handlers
  if (button_01_pressed) {
    ChangeAcquisitionMode();
    button_01_pressed = false;
  }
  if (button_02_pressed) {
    ChangeDataMode();
    button_02_pressed = false;
  }
  // if (button_03_pressed) {
  //   Serial.println("Button 03 Not Attached.");
  //   button_03_pressed = false;
  // }

  DrawToolBar();

  if ((0 == display_mode) and ((current_mode != 0) or (false == screen_initialized))) {
    current_mode == 0;
    screen_initialized = true;
    DrawGraphScreen();
  }
  // else if ((1 == display_mode) and ((current_mode != 1) or (false == screen_initialized))) {
  //   current_mode == 1;
  //   screen_initialized = true;
  //   Serial.println("Data Screen Not Written");
  // }

  if (acquire_data) {
    AcquireData();
  }
}

// Function Definitions
/* BUTTON LOGIC*/
//Button Debounce
void IRAM_ATTR buttonDebounce01() {
  unsigned long current_millis = millis();
  if (current_millis - button_01_last_millis > DEBOUNCE_DELAY) {
    button_01_last_millis = current_millis;
    button_01_pressed = true;
  }
}
void IRAM_ATTR buttonDebounce02() {
  unsigned long current_millis = millis();
  if (current_millis - button_02_last_millis > DEBOUNCE_DELAY) {
    button_02_last_millis = current_millis;
    button_02_pressed = true;
  }
}
// void IRAM_ATTR buttonDebounce03() {
//   unsigned long current_millis = millis();
//   if (current_millis - button_03_last_millis > DEBOUNCE_DELAY) {
//     button_03_last_millis = current_millis;
//     button_03_pressed = true;
//   }
// }
//Button Functions
void ChangeDisplayMode() {
  display_mode++;
  if (display_mode > 1) {
    display_mode = 0;
  }
  Serial.printf("Display Mode: %d\n", display_mode);
}
void ChangeDataMode() {
  data_mode++;
  if (data_mode > 3) {
    data_mode = 1;
  }

  if (2 == data_mode) {
    SAMPLE_FREQ = DATA_FREQ_set1;
  }

  if (3 == data_mode) {
    SAMPLE_FREQ = DATA_FREQ_set2;
  }
  else {
    SAMPLE_FREQ = DEFAULT_SAMPLE_FREQ;
  }
  Serial.printf("Data Mode: %d\n", data_mode);
}
void ChangeAcquisitionMode() {
  acquire_data = !acquire_data;
  Serial.printf("Acquisition Button Pressed. Acquiring: %s\n", acquire_data ? "Yes" : "No");
}

/* TFT SCREEN LOGIC*/

//Welcome Screen
void WriteWelcomeScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  int screenWidth = tft.width();
  int textwidth = strlen("ESP32 Spectrum Analyzer") * 3 * 6;
  int centerX = (screenWidth - textwidth) / 2;
  tft.setCursor(centerX, 100);
  tft.println("ESP32 Spectrum Analyzer");
  tft.setTextSize(2);
  tft.setCursor(10,200);
  tft.println("Group Members:");
  tft.setCursor(10, 240);
  tft.println("Joseph Mesches");
  tft.setCursor(10, 270);
  tft.println("Izaya Trujillo");
  tft.setCursor(10, 300);
  tft.println("Keith Eilor");
  delay(WELCOME_TIME);
}
//Graph Screen
void DrawFrequencyGraph() {
  frequency_graph.createGraph(420,110, TFT_BLACK);
  frequency_graph.setGraphScale(frequency_x_min,
                                frequency_x_max,
                                frequency_y_min,
                                frequency_y_max);
  frequency_graph.setGraphGrid(frequency_x_min,
                               frequency_x_inc,
                               frequency_y_min,
                               frequency_y_inc,
                               FFT_GRID_COLOR);
  frequency_graph.drawGraph(40,40);
  tft.drawLine(39,150,39,40,TFT_WHITE);
  tft.drawLine(38,150,38,40,TFT_WHITE);
  tft.drawLine(39,150,460,150,TFT_WHITE);
  tft.drawLine(39,151,460,151,TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  //Draw FFT Y-Axis Values
  char ymidlabel[5];
  float y_mid = frequency_magnitude_max / 2;
  snprintf(ymidlabel, sizeof(ymidlabel), "%.0f", y_mid);
  int length_y_mid_label = strlen(ymidlabel);
  tft.setCursor((38 - (length_y_mid_label * 1 * 6)) / 2, 96);
  tft.fillRect(0,36,37,124,TFT_BLACK);
  tft.printf("%.0f", y_mid);
  tft.setCursor(16, 146);
  tft.print("0");
  char ymaxlabel[5];
  snprintf(ymaxlabel, sizeof(ymaxlabel), "%.0f", frequency_magnitude_max);
  int length_y_max_label = strlen(ymaxlabel);
  tft.setCursor((38 - (length_y_max_label * 1 * 6)) / 2, 36);
  tft.printf("%.0f", frequency_magnitude_max);
  //Draw FFT X-Axis Values
  float y_max_freq = float(SAMPLE_FREQ / 2);
  for (int i = 0; i < 11; i++) {
    float modifier = (i) / float(10);
    float x_val = y_max_freq * modifier;
    char xlabel[4];
    snprintf(xlabel, sizeof(xlabel), "%.0f", x_val);
    int length_xlabel = strlen(xlabel);
    tft.setCursor((((40 + 42*i) - 21) + ((42 - length_xlabel * 6) / 2)), 155);
    tft.printf("%.0f", x_val);
  }
}
void DrawTimeGraph() {
  timeseries_graph.createGraph(420,110, TFT_BLACK);
  timeseries_graph.setGraphScale(timeseries_x_min,
                                 timeseries_x_max,
                                 timeseries_y_min,
                                 timeseries_y_max);
  timeseries_graph.setGraphGrid(timeseries_x_min,
                                timeseries_x_inc,
                                timeseries_y_min,
                                timeseries_y_inc,
                                TIME_GRID_COLOR);
  timeseries_graph.drawGraph(40,180);
  timeseries_trace.startTrace(TIME_ZERO_COLOR);
  timeseries_trace.addPoint(timeseries_x_min, 0.0);
  timeseries_trace.addPoint(timeseries_x_max, 0.0);
  tft.drawLine(39,290,39,180,TFT_WHITE);
  tft.drawLine(38,290,38,180,TFT_WHITE);
  tft.drawLine(39,290,460,290,TFT_WHITE);
  tft.drawLine(39,291,460,291,TFT_WHITE);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  //Draw TimeSeries Y-Axis Values
  tft.fillRect(0,180,38,140,TFT_BLACK); //Clear Y-Axis
  tft.fillRect(0,291,480,30,TFT_BLACK); //Clear X-Axis
  tft.setCursor(10, 231);
  tft.print("0.0");
  tft.setCursor(10, 286);
  char yminlabel[5];
  snprintf(yminlabel, sizeof(yminlabel), "%.1f", timeseries_y_min);
  int length_y_min_label = strlen(yminlabel);
  tft.setCursor((38 - (length_y_min_label * 1 * 6)) / 2, 286);
  tft.printf("%.1f", timeseries_y_min);
  char ymaxlabel[5];
  snprintf(ymaxlabel, sizeof(ymaxlabel), "%.1f", timeseries_y_max);
  int length_y_max_label = strlen(ymaxlabel);
  tft.setCursor((38 - (length_y_max_label * 1 * 6)) / 2, 176);
  tft.printf("%.1f", timeseries_y_max);
  //Draw TimeSeries X-Axis Values
  float curr_sample_period = (1E6 / SAMPLE_FREQ);
  float max_time = BUFFER_SIZE * (curr_sample_period / 1E6);
  for (int i = 0; i < 11; i++) {
    float modifier = (i) / float(10);
    float x_val = max_time * modifier;
    char xlabel[4];
    snprintf(xlabel, sizeof(xlabel), "%.1f", x_val);
    int length_xlabel = strlen(xlabel);
    tft.setCursor((((40 + 42*i) - 21) + ((42 - length_xlabel * 6) / 2)), 295);
    tft.printf("%.1f", x_val);
  }
  //Start TimeSeries Trace
  timeseries_trace.startTrace(TIME_TRACE_COLOR);
  timeseries_trace.addPoint(0.0, 0.0);
}
void ScaleTimeGraph() {
  float sum_buffer = 0, max_buffer = 0, min_buffer = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum_buffer = sum_buffer + DATA_BUFFER[i];
    if (DATA_BUFFER[i] < min_buffer) {
      min_buffer = DATA_BUFFER[i];
    }
    else if (DATA_BUFFER[i] > max_buffer) {
      max_buffer = DATA_BUFFER[i];
    }
  }
  float buffer_range = (max_buffer - min_buffer);
  if (abs(min_buffer) > max_buffer) {
    max_buffer = abs(min_buffer);
  }
  timeseries_y_min = floor(-1.0*max_buffer);
  timeseries_y_max = ceil(max_buffer);
  timeseries_y_inc = ceil((timeseries_y_max - timeseries_y_min) / 10);
  if (timeseries_y_min == timeseries_y_max) {
    timeseries_y_min = DEFAULT_TIME_Y_MIN;
    timeseries_y_max = DEFAULT_TIME_Y_MAX;
    timeseries_y_inc = DEFAULT_TIME_Y_INC;
  }
  DrawGraphScreen();
}
void ScaleFrequencyGraph();
void PlotFrequencyGraph() {
  int maxVal = 0, minVal = 0;
  for(int i = 0; i < (BUFFER_SIZE / 2); i++) {
    if (DATA_BUFFER[i] > maxVal) {
      maxVal = DATA_BUFFER[i];
    }
  }
  frequency_magnitude_max = maxVal;
  DrawFrequencyGraph();
  frequency_trace.startTrace(FFT_TRACE_COLOR);
  for(int i = 0; i < (BUFFER_SIZE / 2); i++) {
    DATA_BUFFER[i] = 4 * DATA_BUFFER[i] / maxVal; //Magnitude Normalization
    frequency_trace.addPoint(i, DATA_BUFFER[i]);
  }
}
void PlotTimeGraph(int x, int y) {
  timeseries_trace.addPoint(x,y);
  if (x == (BUFFER_SIZE - 1)) {
    ScaleTimeGraph();
    x = 0;
  }
}
//Data Screen
void WriteDataScreen();
//Define Screens
void DrawGraphScreen() {
  tft.fillRect(0, 35, 480, 320, TFT_BLACK);
  DrawTimeGraph();
  DrawFrequencyGraph();
}
void DrawDataScreen();
//Tool Bar
void DrawToolBar() {
  unsigned long current_time = millis();
  if (current_time - last_toolbar_refresh > TOOLBAR_REFRESH_PERIOD) {
    char toolbar_left_update[10];
    char toolbar_center_update[10];
    char toolbar_right_update[10];
    snprintf(toolbar_left_update, sizeof(toolbar_left_update), "%d Hz", SAMPLE_FREQ);
    snprintf(toolbar_center_update, sizeof(toolbar_center_update), "%s", acquire_data ? "Acquiring" : "Stopped");
    if (0 == data_mode) {
      snprintf(toolbar_right_update, sizeof(toolbar_right_update), "%s", "HALL");
    }
    else if (1 == data_mode) {
      snprintf(toolbar_right_update, sizeof(toolbar_right_update), "%s", "ANALOG");
    }
    else if (2 == data_mode) {
      snprintf(toolbar_right_update, sizeof(toolbar_right_update), "%s", "TST: SINE");
    }
    else if (3 == data_mode) {
      snprintf(toolbar_right_update, sizeof(toolbar_right_update), "%s", "TST: EKG");
    }
    tft.setTextSize(2);
    int text_height = 2 * 8;
    int toolbar_height = 30;
    int centerY = (toolbar_height - text_height) /2;
    tft.setTextColor(TOOLBAR_TEXT_COLOR, TOOLBAR_TEXT_BG_COLOR);
    if (strcmp(toolbar_left, toolbar_left_update) != 0) {
      tft.fillRect(0, 0, 160, 30, TOOLBAR_BG_COLOR);
      int textwidth = strlen(toolbar_left_update) * 2 * 6;
      tft.setCursor((160 - textwidth) / 2, centerY);
      tft.print(toolbar_left_update);
      strncpy(toolbar_left, toolbar_left_update, sizeof(toolbar_left) - 1);
      toolbar_left[sizeof(toolbar_left) - 1] = '\0';
    }
    if (strcmp(toolbar_center, toolbar_center_update) != 0) {
      tft.fillRect(160, 0, 160, 30, TOOLBAR_BG_COLOR);
      int textwidth = strlen(toolbar_center_update) * 2 * 6;
      tft.setCursor(160 + ((160 - textwidth) / 2), centerY);
      tft.print(toolbar_center_update);
      strncpy(toolbar_center, toolbar_center_update, sizeof(toolbar_center) - 1);
      toolbar_center[sizeof(toolbar_center) - 1] = '\0';
    }
    if (strcmp(toolbar_right, toolbar_right_update) != 0) {
      tft.fillRect(320, 0, 160, 30, TOOLBAR_BG_COLOR);
      int textwidth = strlen(toolbar_right_update) * 2 * 6;
      tft.setCursor(320 + ((160 - textwidth) / 2), centerY);
      tft.print(toolbar_right_update);
      strncpy(toolbar_right, toolbar_right_update, sizeof(toolbar_right) - 1);
      toolbar_right[sizeof(toolbar_right) - 1] = '\0';
    }
  }
}
/* DATA ACQUISITION LOGIC*/
void AcquireData() {
  unsigned long current_time = micros();
  SAMPLE_PERIOD = 1E6 / SAMPLE_FREQ;
  if (0 == buffer_index) {
    DrawTimeGraph();
  }
  if (current_time - last_acquisition >= SAMPLE_PERIOD) {
    if (sample_start_time != 0) {
      TIME_BUFFER[buffer_index] = 1E6 / (micros() - sample_start_time);
    }
    sample_start_time = micros();
    float data = 0.00;
    last_acquisition = current_time;
    if (0 == data_mode) {
      data = AcquireHall();
    }
    else if (1 == data_mode) {
      data = AcquireAnalog(SIGNAL_PIN);
    }
    else if (2 == data_mode) {
      data = AcquireTest(1);
    }
    else if (3 == data_mode) {
      data = AcquireTest(2);
    }
    WriteBuffer(data);
  }
  return;
}
float AcquireAnalog(unsigned int pin) {
  float raw_analog_value = analogRead(pin);
  float map_analog_value = map(raw_analog_value, 0, 4095, 0.000, 3.300);
  return map_analog_value;
}
float AcquireTest(unsigned int set) {
  unsigned long current_time = micros();
  //Sine Data
  if (1 == set) {
    if (current_time - last_data_point_set1 > DATA_PERIOD_set1) {
      if (data_index_set1 >= 10000) {
        data_index_set1 = 0;
        Serial.println("Reached End of Sine Data.");
      }
      current_data_point = set_one[data_index_set1];
      data_index_set1++;
      last_data_point_set1 = current_time;
    }
  }
  //EKG Data
  else if (2 == set) {
    if (current_time - last_data_point_set2 > DATA_PERIOD_set2) {
      if (data_index_set2 >= 10000) {
        data_index_set2 = 0;
        Serial.println("Reached End of EKG Data.");
      }
      current_data_point = set_two[data_index_set2] * 10.0;
      data_index_set2++;
      last_data_point_set2 = current_time;
    }
  }
  return current_data_point;
}
float AcquireHall() {
  float raw_hall_value = hallRead();
  float mapped_hall_value = map(raw_hall_value, 0, 500, 0.000, 3.300);
  return mapped_hall_value;
}

/* BUFFER LOGIC*/
void ResetBuffers() {
  memset(DATA_BUFFER, 0, BUFFER_SIZE);
  memset(COMPLEX_BUFFER, 0, BUFFER_SIZE);
}
void WriteBuffer(float data) {
  if(buffer_index < BUFFER_SIZE) {
    DATA_BUFFER[buffer_index] = data;
    COMPLEX_BUFFER[buffer_index] = 0;
    PlotTimeGraph(buffer_index, data);
    buffer_index++;
  }
  else {
    acquire_data = false;
    RunFFT();
    buffer_index = 0;
    ResetBuffers();
  }
}

/* FFT LOGIC*/
void RunFFT() {
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.dcRemoval(DATA_BUFFER, BUFFER_SIZE);
  FFT.compute(DATA_BUFFER,
              COMPLEX_BUFFER,
              BUFFER_SIZE,
              BUFFER_POWER,
              FFTDirection::Forward);
  FFT.complexToMagnitude();
  PlotFrequencyGraph();
  float average_sample_freq = 0;
  for(int i=1; i<BUFFER_SIZE; i++) {
    average_sample_freq += TIME_BUFFER[i];
  }
  average_sample_freq /= BUFFER_SIZE;
  Serial.printf("Average Sample Rate: %.2fHz\n", average_sample_freq);
  Serial.printf("Maximum Magnitude: %.0f\n", frequency_magnitude_max);
}

/* LED LOGIC*/
void TurnOffLED();
void SetLEDColor(int color);