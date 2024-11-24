#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Stepper.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define EN_LCD_PIN 16
#define LED_IDLE_PIN 19
#define LED_BUSY_PIN 18
#define LED_ERROR_PIN 5
#define BUTTON_PIN 15         // Simulating error printing
#define BUZZER_PIN 17
#define SERVO_PIN 2
#define RESUME_BUTTON_PIN 4   // Pin for the resume button

#define PRINT_JOB_QUEUE_SIZE 5
#define MAX_PRINT_PAGES 10

#define P_ERROR 0
#define P_POWER_SAVER 1
#define P_IDLE 2
#define P_BUSY 3

const int stepsPerRevolution = 64;
Stepper myStepper(stepsPerRevolution, 26, 27, 32, 33);

QueueHandle_t printJobQueue;
SemaphoreHandle_t printSemaphore;
SemaphoreHandle_t statusSemaphore;

// Task handles
TaskHandle_t printJobTaskHandle = NULL;
TaskHandle_t errorHandlingTaskHandle = NULL;

int AllPrinterStat = P_IDLE;
bool change_priority_receivePrint = false;
char inputBuffer[10];
int bufferIndex = 0;
unsigned long count_idle = 0;
Servo paperFeedServo;

// Task prototypes
void receivePrintJobTask(void *pvParameters);
void printJobTask(void *pvParameters);
void errorHandlingTask(void *pvParameters);
void powerSaverTask(void *pvParameters);
int monitorPrinterStatusTask(int statPrinter);
TaskHandle_t receivePrintJobTaskHandle;

// 'printing_inv', 16x16px
const unsigned char imgprinting_inv [] PROGMEM = {
	0x00, 0x00, 0x1f, 0xf8, 0x10, 0x08, 0x10, 0x08, 0x7f, 0xfe, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xf0, 0x0f, 0xf7, 0xef, 0x10, 0x08, 0x13, 0xc8, 0x1f, 0xf8, 0x00, 0x00
};
// 'wall-clock_inv', 16x16px
const unsigned char imgwall_clock_inv [] PROGMEM = {
	0x00, 0x00, 0x00, 0xf0, 0x00, 0x18, 0x20, 0x04, 0x00, 0x06, 0x00, 0x02, 0x00, 0x02, 0x01, 0x02, 
	0x01, 0x02, 0x06, 0x02, 0x44, 0x02, 0x00, 0x06, 0x20, 0x04, 0x10, 0x18, 0x0f, 0xf0, 0x01, 0x00
};
// 'error_inv', 16x16px
const unsigned char imgerror_inv [] PROGMEM = {
	0x07, 0xe0, 0x1f, 0xf8, 0x3f, 0xfc, 0x7f, 0xfe, 0x7f, 0xfe, 0xfb, 0xdf, 0xfd, 0xbf, 0xfe, 0x7f, 
	0xfe, 0x7f, 0xfd, 0xbf, 0xfb, 0xdf, 0x7f, 0xfe, 0x7f, 0xfe, 0x3f, 0xfc, 0x1f, 0xf8, 0x07, 0xe0
};
// 'warning_inv', 16x16px
const unsigned char imgwarning_inv [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x03, 0xc0, 0x07, 0xe0, 0x06, 0x60, 0x0e, 0x70, 0x0e, 0x70, 
	0x1f, 0xf8, 0x3f, 0xfc, 0x3e, 0x7c, 0x7e, 0x7e, 0x3f, 0xfc, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00
};
// 'energy-saving_inv', 16x16px
const unsigned char imgenergy_saving_inv [] PROGMEM = {
	0x00, 0x00, 0x07, 0xf0, 0x08, 0x0c, 0x10, 0x04, 0x20, 0x82, 0x21, 0xc1, 0x43, 0x61, 0x43, 0x61, 
	0xe2, 0x31, 0xe3, 0x61, 0xe3, 0x61, 0x80, 0x02, 0x00, 0x04, 0x00, 0x8c, 0x00, 0xf0, 0x00, 0x00
};

void setup() {
  Serial.begin(115200);

  pinMode(EN_LCD_PIN, OUTPUT);
  digitalWrite(EN_LCD_PIN, HIGH);
  pinMode(LED_IDLE_PIN, OUTPUT);
  pinMode(LED_BUSY_PIN, OUTPUT);
  pinMode(LED_ERROR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESUME_BUTTON_PIN, INPUT_PULLUP);

  paperFeedServo.attach(SERVO_PIN);
  paperFeedServo.write(0);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }


  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 8);
  display.println("Portable Printer");
  // Display a warning icon at position (40, 20)
  display.drawBitmap(56, 24, imgprinting_inv , 16, 16, WHITE);
  display.display();
  vTaskDelay(1500);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 54);
  display.println("Printer ready...");
  Serial.println("Printer ready...");
  display.display();
  vTaskDelay(300);

  printJobQueue = xQueueCreate(PRINT_JOB_QUEUE_SIZE, sizeof(int));
  printSemaphore = xSemaphoreCreateBinary();
  statusSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(receivePrintJobTask, "Receive Print Job Task", 4096, NULL, 3, &receivePrintJobTaskHandle);
  xTaskCreate(powerSaverTask, "Power Saver Task", 4096, NULL, 1, NULL);
  
  digitalWrite(LED_IDLE_PIN, HIGH);
}

void loop() {
  delay(10);
}

// Task for receiving print jobs
void receivePrintJobTask(void *pvParameters) {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      digitalWrite(EN_LCD_PIN, HIGH);
      if (c == '\n' || bufferIndex >= sizeof(inputBuffer) - 1) {
        inputBuffer[bufferIndex] = '\0';
        bufferIndex = 0;

        int pages = atoi(inputBuffer);
        if (pages > 0 && pages <= MAX_PRINT_PAGES) {
          Serial.printf("Received print job with %d pages.\n", pages);

          if (xQueueSend(printJobQueue, &pages, 0) == pdPASS) {
            Serial.println("Print job added to queue.");

            // Ensure Print Job Task is created
            if (printJobTaskHandle == NULL) {
              xTaskCreate(printJobTask, "Print Job Task", 4096, NULL, 2, &printJobTaskHandle);
              Serial.println("Print Job Task created.");
            }

            // Ensure Error Handling Task is created
            if (errorHandlingTaskHandle == NULL) {
              xTaskCreate(errorHandlingTask, "Error Handling Task", 4096, NULL, 1, &errorHandlingTaskHandle);
              Serial.println("Error Handling Task created.");
            }

            xSemaphoreGive(printSemaphore);
            xSemaphoreGive(statusSemaphore);
          } else {
            Serial.println("Queue full. Cannot add print job.");
          }
        } else {
          Serial.println("Invalid input. Enter a number between 1 and 10.");
        }
      } else {
        inputBuffer[bufferIndex++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void powerSaverTask(void *pvParameters) {
  while (true) {
    if(digitalRead(RESUME_BUTTON_PIN) == LOW){
        if(AllPrinterStat == P_POWER_SAVER){
          display.fillRect(0, 40, 128, 24, BLACK);
          display.display();
          display.setCursor(0, 54);
          display.print("Standby...");
          display.display();
          AllPrinterStat = monitorPrinterStatusTask(P_IDLE);
          count_idle = millis(); 
        }    
    }
    else if((AllPrinterStat == P_IDLE)||(AllPrinterStat == P_IDLE)){
      unsigned long current_millis = millis();
      Serial.println("count idle");
      Serial.println(count_idle);
      Serial.println(current_millis);
      if (current_millis - count_idle > 5000){
        Serial.println("Printer enter power saver mode.");
        display.fillRect(0, 40, 128, 24, BLACK);
        display.display();
        display.setCursor(0, 54);
        display.print("Enter power saver");
        display.display();
        AllPrinterStat = monitorPrinterStatusTask(P_POWER_SAVER);  
      } 
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// Task for processing print jobs
void printJobTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(printSemaphore, portMAX_DELAY) == pdTRUE) {
      int pages = 0;
      if (xQueueReceive(printJobQueue, &pages, portMAX_DELAY) == pdPASS) {
        AllPrinterStat = monitorPrinterStatusTask(P_BUSY);

        Serial.println("Printing job started...");
        // Clears a 16x16 area at position (56, 24)
        display.fillRect(0, 40, 128, 24, BLACK);
        display.display();
        display.setCursor(0, 54);
        display.print("Printing ");
        display.print(pages);
        display.println(" pages...");
        display.display();

        // Feed paper only once before printing
        Serial.println("Feeding paper...");
        paperFeedServo.write(90);
        vTaskDelay(pdMS_TO_TICKS(1000));
        paperFeedServo.write(0);

        // Simulate printing pages
        for (int i = 1; i <= pages; i++) {
          while (AllPrinterStat ==  P_ERROR) {
            Serial.println("Printing paused due to error. Waiting for resume...");
            display.fillRect(0, 40, 128, 24, BLACK);
            display.display();
            display.setCursor(0, 54);
            display.println("Error! Printing paused.");
            display.display();              
            // Wait for resume signal from resume button (polling it)
            while (digitalRead(RESUME_BUTTON_PIN) == HIGH) {
              vTaskDelay(pdMS_TO_TICKS(100));  // Polling every 100 ms
            }

            // Once resume button is pressed, clear the error state and continue printing
            AllPrinterStat = monitorPrinterStatusTask(P_BUSY);
            Serial.println("Resuming print job...");

            // Move the servo to simulate paper feed when resuming after an error
            paperFeedServo.write(90);  // Simulate paper feed movement
            vTaskDelay(pdMS_TO_TICKS(1000));  // Paper feed delay
            paperFeedServo.write(0);   // Reset servo

            display.fillRect(0, 40, 128, 24, BLACK);
            display.display();
            display.setCursor(0, 54);
            display.print("Resuming...");
            display.display();
          }
          Serial.printf("Printing page %d\n", i);
          display.fillRect(0, 40, 128, 24, BLACK);
          display.display();
          display.setCursor(0, 54);
          display.print("Page ");
          display.print(i);
          display.println(" printed.");
          display.display();
          myStepper.step(stepsPerRevolution);
          vTaskDelay(pdMS_TO_TICKS(500));
        }

        // After printing is done, set the printer to idle
        Serial.println("Print job complete.");

        count_idle = millis();
        AllPrinterStat = monitorPrinterStatusTask(P_IDLE);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

int monitorPrinterStatusTask(int printerStat) {
  //=========printer error==============================
  if (printerStat == P_ERROR) {
    Serial.println("Error: Printer has a problem!");
    digitalWrite(LED_IDLE_PIN, LOW);
    digitalWrite(LED_ERROR_PIN, HIGH);
    digitalWrite(LED_BUSY_PIN, LOW);
    digitalWrite(EN_LCD_PIN, HIGH);
    tone(BUZZER_PIN, 1000, 500);  // Buzzer for error
    
    display.fillRect(0, 0, 128, 40, BLACK);
    display.display();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 8);
    display.println("Printer Error");
    display.drawBitmap(56, 24, imgerror_inv, 16, 16, WHITE);
    display.display();
  //=========printer error==============================
  } 
  else if (printerStat == P_POWER_SAVER){
      if(!change_priority_receivePrint){
        Serial.println("change receivePrintJobTaskHandle to 1");
        change_priority_receivePrint = true;
        vTaskPrioritySet(receivePrintJobTaskHandle, 1);
      } 
      Serial.println("Turn off some component");
      digitalWrite(LED_IDLE_PIN, LOW);
      digitalWrite(LED_ERROR_PIN, LOW);
      digitalWrite(LED_BUSY_PIN, LOW);
      digitalWrite(EN_LCD_PIN, LOW);
      display.fillRect(0, 0, 128, 40, BLACK);
      display.display();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(20, 8);
      display.println("Power Saver Mode");
      display.drawBitmap(56, 24, imgenergy_saving_inv, 16, 16, WHITE);
      display.display();
      if (uxQueueMessagesWaiting(printJobQueue) == 0) {
        Serial.println("No more jobs in queue. Deleting tasks...");
        if (printJobTaskHandle != NULL) {
          vTaskDelete(printJobTaskHandle);
          printJobTaskHandle = NULL;
          Serial.println("Deleting tasks printJob...");
        }
        if (errorHandlingTaskHandle != NULL) {
          vTaskDelete(errorHandlingTaskHandle);
          errorHandlingTaskHandle = NULL;
          Serial.println("Deleting tasks errorHandling...");
        }
      }
  }
  else if (printerStat == P_IDLE) {
    Serial.println("Printer is idle.");
    digitalWrite(LED_ERROR_PIN, LOW);
    digitalWrite(LED_BUSY_PIN, LOW);
    digitalWrite(LED_IDLE_PIN, HIGH);
    digitalWrite(EN_LCD_PIN, HIGH);

    display.fillRect(0, 0, 128, 40, BLACK);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 8);
    display.println("Printer Standby");
    display.drawBitmap(56, 24, imgwall_clock_inv, 16, 16, WHITE);
    display.display();
  }       
  else if (printerStat == P_BUSY) {
    Serial.println("Printer is busy...");
    digitalWrite(LED_ERROR_PIN, LOW);
    digitalWrite(LED_BUSY_PIN, HIGH);
    digitalWrite(LED_IDLE_PIN, LOW);
    digitalWrite(EN_LCD_PIN, HIGH);

    display.fillRect(0, 0, 128, 40, BLACK);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 8);
    display.println("Printing...");
    display.drawBitmap(56, 24, imgprinting_inv, 16, 16, WHITE);
    display.display();

    if(change_priority_receivePrint){
      Serial.println("change receivePrintJobTaskHandle to 3");
      vTaskPrioritySet(receivePrintJobTaskHandle, 3);
      change_priority_receivePrint = false;
    }
  }
  return printerStat;
}

// Task for error handling
void errorHandlingTask(void *pvParameters) {
  while (true) {
    if ((digitalRead(BUTTON_PIN) == LOW)&&(AllPrinterStat==P_BUSY)) {
      AllPrinterStat = monitorPrinterStatusTask(P_ERROR);
    }
    if((digitalRead(RESUME_BUTTON_PIN) == LOW)&&(AllPrinterStat == P_ERROR)){
      AllPrinterStat = monitorPrinterStatusTask(P_BUSY);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
