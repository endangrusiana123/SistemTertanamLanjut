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
#define PAPER_SENS_PIN 15            // Simulating error printing
#define INKLEVEL_SENS_PIN 23         // Simulating error printing
#define BUZZER_PIN 17
#define SERVO_PIN 2
#define RESUME_BTN_PIN 4   // Pin for the resume button

#define PRINT_JOB_QUEUE_SIZE 5
#define MAX_PRINT_PAGES 10

#define P_IDLE 0
#define P_ERROR 1
#define P_POWER_SAVER 2
#define P_BUSY 3
#define P_WARNING 4

#define STEPS_PER_UNIT 4         // Steps per unit of movement (calibrate as needed)
// Speed options (delay in milliseconds)
#define SLOW_DELAY 10
#define MEDIUM_DELAY 6
#define FAST_DELAY 1
#define SPEED_SLOW 1
#define SPEED_MEDIUM 2
#define SPEED_FAST 3
#define END_POS_P_HEAD 4
#define PAPER_LINE 2

#define MODE_PAPER 0
#define MODE_IMAGE 1

const int stepsPerRevolution = 64;
Stepper myStepper(stepsPerRevolution, 26, 27, 32, 33);

bool change_priority_receivePrint = false;
char inputBuffer[10];
int bufferIndex = 0;
unsigned long count_idle = 0;
Servo paperFeedServo;
int printerStat2Disp = P_IDLE;

typedef struct {
    int pages;           // Number of pages
    int mode;            // Printing mode (e.g., image or paper)
} structPrintJobQueue;

QueueHandle_t printJobQueue;
SemaphoreHandle_t printSemaphore;
SemaphoreHandle_t displaySemaphore;
SemaphoreHandle_t statusPrinterSemaphore;

// Task handles
TaskHandle_t printJobTaskHandle = NULL;
TaskHandle_t receivePrintJobTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t statusMonitoringTaskHandle = NULL;

// Task prototypes
void receivePrintJobTask(void *pvParameters);
void printJobTask(void *pvParameters);
void powerSaverTask(void *pvParameters);
void displayTask(void *pvParameters);
void statusMonitoringTask(void *pvParameters);
// Function to move the motor to a target position incrementally
void moveStepperToPosition(int targetPosition, int speedMode);
void errorCheck();
void oledDisplayInit();
void displayCreateTask();

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

int index_img = P_IDLE;

const char* TXT_IDLE = "Idle";
const char* TXT_ERROR = "Error";
const char* TXT_POWER_SAVER = "Power Saver";
const char* TXT_PRINTING = "Printing";
const char* TXT_STANDBY = "Standby..";
const char* TXT_ENTER_POWER_SAVER = "Enter power saver";
const char* TXT_RESUMING = "Resuming..";


// Message structure
struct DisplayMessage {
    String line1;
    String line2;
};

DisplayMessage message;

void setup() {
  Serial.begin(115200);

  pinMode(EN_LCD_PIN, OUTPUT);
  digitalWrite(EN_LCD_PIN, HIGH);
  pinMode(LED_IDLE_PIN, OUTPUT);
  pinMode(LED_BUSY_PIN, OUTPUT);
  pinMode(LED_ERROR_PIN, OUTPUT);
  pinMode(PAPER_SENS_PIN, INPUT_PULLUP);
  pinMode(INKLEVEL_SENS_PIN, INPUT_PULLUP);
  pinMode(RESUME_BTN_PIN, INPUT_PULLUP);

  oledDisplayInit();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 8);
  display.println("Portable Printer");
  // Display a warning icon at position (40, 20)
  display.drawBitmap(56, 24, imgprinting_inv , 16, 16, WHITE);
  display.display();
  vTaskDelay(pdMS_TO_TICKS(1500));

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 54);
  display.println("Printer setup");
  Serial.println("Printer setup");
  display.display();
  vTaskDelay(pdMS_TO_TICKS(300));

  paperFeedServo.attach(SERVO_PIN);
  paperFeedServo.write(0);
  // Set motor speed (maximum speed; actual speed controlled via delay)
  myStepper.setSpeed(10);

  printJobQueue = xQueueCreate(PRINT_JOB_QUEUE_SIZE, sizeof(structPrintJobQueue));
  printSemaphore = xSemaphoreCreateBinary();
  displaySemaphore = xSemaphoreCreateBinary();
  statusPrinterSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(displaySemaphore);
  xSemaphoreGive(statusPrinterSemaphore);

  xTaskCreate(receivePrintJobTask, "Receive Print Job Task", 4096, NULL, 3, &receivePrintJobTaskHandle);
  Serial.println("task1 created..");
  xTaskCreate(displayTask, "Display Task", 4096, NULL, 1, &displayTaskHandle);
  Serial.println("task2 created..");
  xTaskCreate(statusMonitoringTask, "Status Monitoring Task", 4096, NULL, 1, &statusMonitoringTaskHandle);
  Serial.println("task3 created..");
  xTaskCreate(powerSaverTask, "Power Saver Task", 4096, NULL, 1, NULL);
  
  digitalWrite(LED_IDLE_PIN, HIGH);
  Serial.println("Printer ready...");
}

void loop() {
  delay(10);
}

void displayCreateTask(){
  if (displayTaskHandle == NULL) {
    xTaskCreate(displayTask, "Display Task", 4096, NULL, 1, &displayTaskHandle);
    Serial.println("Display Task created.");
  }
}

void oledDisplayInit(){
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
}

// Task for receiving print jobs
void receivePrintJobTask(void *pvParameters) {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || bufferIndex >= sizeof(inputBuffer) - 1) {
        inputBuffer[bufferIndex] = '\0';
        bufferIndex = 0;

        // Parse the input buffer
        char *token = strtok(inputBuffer, ",");
        if (token == NULL) {
            Serial.println("Invalid input format. Use: <pages>,<mode>");
            continue;
        }

        // Extract the number of pages
        int pages = atoi(token);
        if (pages <= 0 || pages > MAX_PRINT_PAGES) {
            Serial.println("Invalid page count. Enter a number between 1 and 10.");
            continue;
        }

        // Extract the printing mode
        token = strtok(NULL, ",");
        if (token == NULL) {
            Serial.println("Invalid input format. Missing printing mode.");
            continue;
        }

        String printingMode = String(token);  // Convert to String for comparison
        // Validate printing mode
        if (printingMode != "image" && printingMode != "paper") {
            Serial.println("Invalid printing mode. Use: image or paper.");
            continue;
        }

        structPrintJobQueue job;
        job.pages = pages;
        job.mode = (printingMode == "image") ? MODE_IMAGE : MODE_PAPER;

        Serial.printf("Received print job: %d pages, mode: %s\n", pages, token);

        if (xQueueSend(printJobQueue, &job, 0) == pdPASS) {
          Serial.println("Print job added to queue.");
          displayCreateTask();
          // Ensure Print Job Task is created
          if (printJobTaskHandle == NULL) {
            xTaskCreate(printJobTask, "Print Job Task", 4096, NULL, 2, &printJobTaskHandle);
            Serial.println("Print Job Task created.");
          }
          xSemaphoreGive(printSemaphore);
        } else {
          Serial.println("Queue full. Cannot add print job.");
        }
      } else {
        inputBuffer[bufferIndex++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void powerSaverTask(void *pvParameters) {
int idleStat = 0;
  while (true) {
    if (xSemaphoreTake(statusPrinterSemaphore, portMAX_DELAY) == pdTRUE) {
      if(digitalRead(RESUME_BTN_PIN) == LOW){
        if(printerStat2Disp == P_POWER_SAVER){
          displayCreateTask();
          printerStat2Disp = P_IDLE;
          while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          message.line1 = TXT_IDLE;
          message.line2 = TXT_STANDBY;
          index_img = P_IDLE;
          xSemaphoreGive(displaySemaphore);

          count_idle = millis(); 
        }   
      }
      else if(printerStat2Disp == P_IDLE){
        unsigned long current_millis = millis();
        Serial.println("count idle");
        Serial.println(count_idle);
        Serial.println(current_millis);
        if (current_millis - count_idle > 5000){
          Serial.println("Printer enter power saver mode.");
          printerStat2Disp = P_POWER_SAVER;
          while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
            Serial.println("wait display");
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          message.line1 = TXT_POWER_SAVER;
          message.line2 = TXT_ENTER_POWER_SAVER;
          index_img = P_POWER_SAVER;
          xSemaphoreGive(displaySemaphore);
        }
        else if(idleStat == 0){
          while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          message.line1 = TXT_IDLE;
          message.line2 = TXT_STANDBY;
          index_img = P_IDLE;
          xSemaphoreGive(displaySemaphore);
          idleStat = 1;
        }
      }
      else{
        idleStat = 0;
      }
      xSemaphoreGive(statusPrinterSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Function to move the motor to a target position incrementally
void moveStepperToPosition(int targetPosition, int speedMode) {
    static int currentPosition = 0;  // Tracks the current position of the motor
    int stepDelay;

    // Set delay based on speed mode
    switch (speedMode) {
        case 1: stepDelay = SLOW_DELAY; break;    // Slow speed
        case 2: stepDelay = MEDIUM_DELAY; break;  // Medium speed
        case 3: stepDelay = FAST_DELAY; break;    // Fast speed
        default: 
            Serial.println("Invalid speed mode! Defaulting to medium speed.");
            stepDelay = MEDIUM_DELAY;
    }

    Serial.printf("Moving from %d to %d at %s speed\n", 
                  currentPosition, 
                  targetPosition, 
                  (speedMode == 1) ? "slow" : (speedMode == 2) ? "medium" : "fast");

    // Calculate the number of steps required
    int stepsToMove = (targetPosition - currentPosition) * STEPS_PER_UNIT;
    int stepDirection = (stepsToMove > 0) ? 1 : -1;

    // Move incrementally
    while (currentPosition != targetPosition) {
        myStepper.step(stepDirection * STEPS_PER_UNIT);  // Move one step
        currentPosition += stepDirection;             // Update position
        delay(stepDelay);                             // Delay for speed control

        Serial.printf("Current Position: %d\n", currentPosition);  // Log position
        errorCheck();
    }

    Serial.println("Reached target position.");
}

void errorCheck(){
String error_text = "";
  if (((digitalRead(PAPER_SENS_PIN) == LOW) || (digitalRead(INKLEVEL_SENS_PIN) == LOW))) { 
    error_text ="";    
    if (digitalRead(PAPER_SENS_PIN) == LOW) {
      error_text += "Paper empty";
    }
    if(error_text != ""){
      error_text += ", ";
    }
    // Check for ink error
    if (digitalRead(INKLEVEL_SENS_PIN) == LOW) {
      error_text += "Ink low"; // Add ink error
    }
    while(xSemaphoreTake(statusPrinterSemaphore, 0) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(100));
    };
    printerStat2Disp = P_ERROR;
    xSemaphoreGive(statusPrinterSemaphore);

    while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    message.line1 = TXT_ERROR;
    message.line2 = error_text;
    index_img = P_ERROR;
    xSemaphoreGive(displaySemaphore);

    Serial.println("Printing paused due to error. Waiting for resume...");
    // Wait for resume signal from resume button (polling it)
    while (digitalRead(RESUME_BTN_PIN) == HIGH) {
      vTaskDelay(pdMS_TO_TICKS(100));  // Polling every 100 ms
    }

    // Once resume button is pressed, clear the error state and continue printing
    while(xSemaphoreTake(statusPrinterSemaphore, 0) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(100));
    };
    printerStat2Disp = P_BUSY;
    xSemaphoreGive(statusPrinterSemaphore);

    while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    message.line1 = TXT_PRINTING;
    message.line2 = TXT_RESUMING;
    index_img = P_BUSY;
    xSemaphoreGive(displaySemaphore);
    Serial.println("Resuming print job...");
  }
}

// Task for processing print jobs
void printJobTask(void *pvParameters) {
int printingMode = MODE_PAPER;
structPrintJobQueue job;
char buffer[16];
  while (true) {
    if (xSemaphoreTake(printSemaphore, portMAX_DELAY) == pdTRUE) {
      int pages = 0;
      if (xQueueReceive(printJobQueue, &job, portMAX_DELAY) == pdPASS) {
        pages = job.pages;
        printingMode = job.mode;
        UBaseType_t messagesWaiting = uxQueueMessagesWaiting(printJobQueue);
        sprintf(buffer,"Printing %d pages, Q:%d",pages,messagesWaiting);
        while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        message.line1 = TXT_PRINTING;
        message.line2 = buffer;
        index_img = P_BUSY;
        xSemaphoreGive(displaySemaphore);
        while(xSemaphoreTake(statusPrinterSemaphore, 0) != pdTRUE) {
          vTaskDelay(pdMS_TO_TICKS(100));
        };
        printerStat2Disp = P_BUSY;
        xSemaphoreGive(statusPrinterSemaphore);
        
        Serial.println("Printing job started...");

        // Feed paper only once before printing
        Serial.println("Feeding paper...");
        paperFeedServo.write(90);
        vTaskDelay(pdMS_TO_TICKS(1000));
        paperFeedServo.write(0);

        // Simulate printing pages
        for (int i = 1; i <= pages; i++) {
          // Move the servo to simulate paper feed when resuming after an error
          paperFeedServo.write(90);  // Simulate paper feed movement
          vTaskDelay(pdMS_TO_TICKS(1000));  // Paper feed delay
          paperFeedServo.write(0);   // Reset servo

          Serial.printf("Printing page %d\n", i);
          
          sprintf(buffer,"Page %d printed",i);
          while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          message.line1 = TXT_PRINTING;
          message.line2 = buffer;
          index_img = P_BUSY;
          xSemaphoreGive(displaySemaphore);

          for(int line=0;line<PAPER_LINE;line++){
            if(printingMode == MODE_IMAGE){
              moveStepperToPosition(END_POS_P_HEAD, SPEED_SLOW);
            }else if(printingMode == MODE_PAPER){
              moveStepperToPosition(END_POS_P_HEAD, SPEED_MEDIUM); 
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            moveStepperToPosition(0, SPEED_FAST);   // Fast mode
            vTaskDelay(pdMS_TO_TICKS(500));

            paperFeedServo.write(90);  // Simulate paper feed movement
            vTaskDelay(pdMS_TO_TICKS(1000));  // Paper feed delay
            paperFeedServo.write(0);   // Reset servo
          }
        }
        // After printing is done, set the printer to idle
        Serial.println("Print job complete.");

        if(messagesWaiting == 0){
          count_idle = millis();
          while(xSemaphoreTake(statusPrinterSemaphore, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(100));
          };
          printerStat2Disp = P_IDLE;
          xSemaphoreGive(statusPrinterSemaphore);
          while(xSemaphoreTake(displaySemaphore, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
          message.line1 = TXT_IDLE;
          message.line2 = TXT_STANDBY;
          index_img = P_IDLE;
          xSemaphoreGive(displaySemaphore);
        }else{
          xSemaphoreGive(printSemaphore);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void displayTask(void *pvParameters) {
    // Local variables for display updates
    String localLine1 = "";
    String localLine2 = "";
    int localIndexImg = P_IDLE;
    int initStat = 0;
    while (true) {
        // Take the semaphore to access shared resources
        if (xSemaphoreTake(displaySemaphore, portMAX_DELAY) == pdTRUE) {
            // Safely copy shared data to local variables
            localLine1 = message.line1;
            localLine2 = message.line2;
            localIndexImg = index_img;

            // Release semaphore after copying
            xSemaphoreGive(displaySemaphore);
        }
        if(initStat == 0){
          oledDisplayInit();
          initStat = 1;
        }
        else{
          // Clear the display areas
          display.fillRect(0, 0, 128, 40, BLACK);
          display.fillRect(0, 40, 128, 24, BLACK);

          // Update the display
          display.setTextColor(SSD1306_WHITE);
          display.setTextSize(1);
          display.setCursor(20, 8);
          display.println(localLine1);

          switch (localIndexImg) {
            case P_IDLE:
              // Draw the bitmap image
              display.drawBitmap(56, 24, imgwall_clock_inv, 16, 16, WHITE);
            break;
            case P_ERROR:
              // Draw the bitmap image
              display.drawBitmap(56, 24, imgerror_inv, 16, 16, WHITE);
            break;
            case P_POWER_SAVER:
              // Draw the bitmap image
              display.drawBitmap(56, 24, imgenergy_saving_inv, 16, 16, WHITE);
            break;
            case P_BUSY:
              // Draw the bitmap image
              display.drawBitmap(56, 24, imgprinting_inv, 16, 16, WHITE);
            break;
          }

          // Display the second line of text
          display.setCursor(0, 54);
          display.print(localLine2);

          // Push updates to the display
          display.display();

        }
        // Delay for 300 ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void statusMonitoringTask(void *pvParameters) {
int LocalPrinterStat2Disp = P_IDLE;
int PrevLocalPrinterStat2Disp = P_IDLE;
  while(true){
    if (xSemaphoreTake(statusPrinterSemaphore, portMAX_DELAY) == pdTRUE) {
      LocalPrinterStat2Disp = printerStat2Disp;
      xSemaphoreGive(statusPrinterSemaphore);
    }
    if(LocalPrinterStat2Disp != PrevLocalPrinterStat2Disp){
     //=========printer error==============================
      if (printerStat2Disp == P_ERROR) {
        Serial.println("Error: Printer has a problem!");
        digitalWrite(LED_IDLE_PIN, LOW);
        digitalWrite(LED_ERROR_PIN, HIGH);
        digitalWrite(LED_BUSY_PIN, LOW);
        digitalWrite(EN_LCD_PIN, HIGH);
        tone(BUZZER_PIN, 1000, 500);  // Buzzer for error
      //=========printer error==============================
      } 
      else if (printerStat2Disp == P_POWER_SAVER){
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
          if (uxQueueMessagesWaiting(printJobQueue) == 0) {
            Serial.println("No more jobs in queue. Deleting tasks...");
            if (printJobTaskHandle != NULL) {
              vTaskDelete(printJobTaskHandle);
              printJobTaskHandle = NULL;
              Serial.println("Deleting tasks printJob...");
            }
            if (displayTaskHandle != NULL) {
              vTaskDelete(displayTaskHandle);
              displayTaskHandle = NULL;
              Serial.println("Deleting tasks display...");
            }
          }
      }
      else if (printerStat2Disp == P_IDLE) {
        Serial.println("Printer is idle.");
        digitalWrite(LED_ERROR_PIN, LOW);
        digitalWrite(LED_BUSY_PIN, LOW);
        digitalWrite(LED_IDLE_PIN, HIGH);
        digitalWrite(EN_LCD_PIN, HIGH);
      }       
      else if (printerStat2Disp == P_BUSY) {
        Serial.println("Printer is busy...");
        digitalWrite(LED_ERROR_PIN, LOW);
        digitalWrite(LED_BUSY_PIN, HIGH);
        digitalWrite(LED_IDLE_PIN, LOW);
        digitalWrite(EN_LCD_PIN, HIGH);
        if(change_priority_receivePrint){
          Serial.println("change receivePrintJobTaskHandle to 3");
          vTaskPrioritySet(receivePrintJobTaskHandle, 3);
          change_priority_receivePrint = false;
        }
      }
      PrevLocalPrinterStat2Disp = LocalPrinterStat2Disp;
    }
    // Delay for 100 ms
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

