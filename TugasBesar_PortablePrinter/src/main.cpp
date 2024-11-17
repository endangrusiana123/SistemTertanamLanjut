#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Stepper.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define LED_IDLE_PIN 19
#define LED_BUSY_PIN 18
#define LED_ERROR_PIN 5
#define BUTTON_PIN 15         // Simulating error printing
#define BUZZER_PIN 17
#define SERVO_PIN 2
#define RESUME_BUTTON_PIN 4   // Pin for the resume button

#define PRINT_JOB_QUEUE_SIZE 5
#define MAX_PRINT_PAGES 10

#define DEB_RECEIVE_PRINT_JOB 14
#define DEB_PRINT_JOB_TASK 27
#define DEB_MONITOR_PRINTER_STAT 32
#define DEB_PAPER_FEED 33
#define DEB_ERROR_HANDLING 25
#define SERIAL_INPUT 26

const int stepsPerRevolution = 64;  // change this to fit the number of steps per revolution
// for your motor
// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 26, 27, 32, 33);

// Queues
QueueHandle_t printJobQueue;
SemaphoreHandle_t printSemaphore;
SemaphoreHandle_t statusSemaphore;
SemaphoreHandle_t inputSemaphore;  // Semaphore to signal new input

// Printer Status
bool printerIdle = true;
bool printerError = false;
int printJobPages = 0;

// Serial input buffer
char inputBuffer[10];  // Buffer to hold incoming serial data
int bufferIndex = 0;   // Index for input buffer

// Create Servo object
Servo paperFeedServo;

// Task Prototypes
void receivePrintJobTask(void *pvParameters);
void printJobTask(void *pvParameters);
void monitorPrinterStatusTask(void *pvParameters);
void paperFeedTask(void *pvParameters);
void errorHandlingTask(void *pvParameters);
void serialInputTask(void *pvParameters);

void setup() {
  Serial.begin(115200);

  pinMode(LED_IDLE_PIN, OUTPUT);
  pinMode(LED_BUSY_PIN, OUTPUT);
  pinMode(LED_ERROR_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESUME_BUTTON_PIN, INPUT_PULLUP);  // Set the resume button pin as input with pull-up
  pinMode(BUZZER_PIN, OUTPUT);

  // set the speed at 60 rpm:
  myStepper.setSpeed(60);

  paperFeedServo.attach(SERVO_PIN);
  paperFeedServo.write(0);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Printer ready...");
  display.display();
  Serial.println("Printer ready...");

  printJobQueue = xQueueCreate(PRINT_JOB_QUEUE_SIZE, sizeof(int));
  printSemaphore = xSemaphoreCreateBinary();
  statusSemaphore = xSemaphoreCreateBinary();
  inputSemaphore = xSemaphoreCreateBinary();  // Create semaphore for serial input

  // Create Tasks
  xTaskCreate(receivePrintJobTask, "Receive Print Job", 4096, NULL, 3, NULL);
  xTaskCreate(printJobTask, "Print Job Task", 4096, NULL, 2, NULL);
  xTaskCreate(monitorPrinterStatusTask, "Monitor Printer Status", 4096, NULL, 1, NULL);
  xTaskCreate(paperFeedTask, "Paper Feed Task", 4096, NULL, 1, NULL);
  xTaskCreate(errorHandlingTask, "Error Handling Task", 4096, NULL, 1, NULL);
  xTaskCreate(serialInputTask, "Serial Input Task", 4096, NULL, 1, NULL);
}

void loop() {
  // Nothing in loop, all handled by tasks
  delay(10);
}

// Task for reading serial input and storing it into a buffer
void serialInputTask(void *pvParameters) {
  while (true) {
    if (Serial.available()) {

      char c = Serial.read();  // Read character from serial
      if (c == '\n' || bufferIndex >= sizeof(inputBuffer) - 1) {  // End of input (newline or buffer full)
        inputBuffer[bufferIndex] = '\0';  // Null-terminate the string
        bufferIndex = 0;  // Reset the buffer index

        // Signal the receivePrintJobTask that new data is available
        xSemaphoreGive(inputSemaphore);
      } else {
        inputBuffer[bufferIndex++] = c;  // Add character to buffer
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // Small delay to avoid overloading the processor
  }
}

// Task 1: Process Print Job Command from serial input
void receivePrintJobTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(inputSemaphore, portMAX_DELAY) == pdTRUE) {
      // Convert inputBuffer to integer
      int pages = atoi(inputBuffer);

      if (pages > 0 && pages <= MAX_PRINT_PAGES) {
        Serial.print("Received print job with ");
        Serial.print(pages);
        Serial.println(" pages. Available Queue:");

        int availableSpace = uxQueueSpacesAvailable(printJobQueue);
        Serial.println(availableSpace);
        // Send the print job (number of pages) to the print queue
        if (xQueueSend(printJobQueue, &pages, 0) == pdPASS) {
          Serial.println("Print job added to queue.");
          xSemaphoreGive(printSemaphore);  // Notify Print Task to start
        } else {
          Serial.println("Failed to add print job to queue.");
        }
      } else if (pages != 0) {
        Serial.print(pages);
        Serial.println(" Invalid input. Please enter a number between 1 and 10.");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));  // Wait before checking for input again
  }
}

// Task 2: Simulate the print job processing
void printJobTask(void *pvParameters) {
  while (true) {
    // Wait for a valid print job to arrive
    if (xSemaphoreTake(printSemaphore, portMAX_DELAY) == pdTRUE) {
      int pages = 0;  // Use local variable to hold the number of pages from the queue
      if (xQueueReceive(printJobQueue, &pages, portMAX_DELAY) == pdPASS) {
        if (printerIdle) {
          printerIdle = false;
          digitalWrite(LED_BUSY_PIN, HIGH);
          digitalWrite(LED_IDLE_PIN, LOW);

          display.clearDisplay();
          display.setCursor(0, 0);
          display.print("Printing ");
          display.print(pages);  // Use the value from the queue
          display.println(" pages...");
          display.display();
          Serial.println("Printing...");
          vTaskDelay(pdMS_TO_TICKS(1000));

          // Simulate printing pages
          for (int i = 1; i <= pages; i++) {
            // Check if there's an error before proceeding
            while (printerError) {
              Serial.println("Printing paused due to error. Waiting for resume...");
              display.clearDisplay();
              display.setCursor(0, 0);
              display.println("Error! Printing paused.");
              display.display();              
              // Wait for resume signal from resume button (polling it)
              while (digitalRead(RESUME_BUTTON_PIN) == HIGH) {
                vTaskDelay(pdMS_TO_TICKS(100));  // Polling every 100 ms
              }

              // Once resume button is pressed, clear the error state and continue printing
              printerError = false;
              digitalWrite(LED_ERROR_PIN, LOW);
              Serial.println("Resuming print job...");

              // Move the servo to simulate paper feed when resuming after an error
              paperFeedServo.write(90);  // Simulate paper feed movement
              vTaskDelay(pdMS_TO_TICKS(1000));  // Paper feed delay
              paperFeedServo.write(0);   // Reset servo

              display.clearDisplay();
              display.setCursor(0, 0);
              display.print("Resuming...");
              display.display();
            }

            // Continue printing
            Serial.print("Printing page ");
            Serial.println(i);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.print("Page ");
            display.print(i);
            display.println(" printed.");
            display.display();
            myStepper.step(stepsPerRevolution);
            vTaskDelay(pdMS_TO_TICKS(300));  // Reduced delay to avoid long freezes
          }
          // After printing is done, set the printer to idle
          Serial.println("Print job complete.");
          digitalWrite(LED_BUSY_PIN, LOW);
          digitalWrite(LED_IDLE_PIN, HIGH);

          printerIdle = true;
          xSemaphoreGive(statusSemaphore);  // Notify Status Task

          // Ensure the semaphore is given so that the next print job can start
          xSemaphoreGive(printSemaphore);
          
          // Add a small delay to avoid immediate paper feed
          vTaskDelay(pdMS_TO_TICKS(2000));  // Wait before notifying the paper feed task
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Periodic check
  }
}

// Task 3: Simulate printer status monitoring
void monitorPrinterStatusTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(statusSemaphore, portMAX_DELAY) == pdTRUE) {
      if (printerError) {
        Serial.println("Error: Printer has a problem!");
        digitalWrite(LED_ERROR_PIN, HIGH);
        tone(BUZZER_PIN, 1000, 500);  // Buzzer for error
      } else if (printerIdle) {
        Serial.println("Printer is idle.");
        digitalWrite(LED_ERROR_PIN, LOW);
        digitalWrite(LED_IDLE_PIN, HIGH);
      } else {
        Serial.println("Printer is busy...");
        digitalWrite(LED_BUSY_PIN, HIGH);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Periodic check
  }
}

// Task 4: Simulate paper feed mechanism
void paperFeedTask(void *pvParameters) {
  bool lastPrinterIdleState = false;  // Track last idle state
  while (true) {
    if (printerIdle && !lastPrinterIdleState) {
      // Only feed paper when the printer goes from busy to idle (one-time feed)
      Serial.println("Feeding paper for next job...");
      paperFeedServo.write(90);  // Simulate paper feed movement
      vTaskDelay(pdMS_TO_TICKS(1000));  // Paper feed delay
      paperFeedServo.write(0);   // Reset servo
      vTaskDelay(pdMS_TO_TICKS(1000));  // Paper feed delay
      
      lastPrinterIdleState = true;
      Serial.println("Paper feeder move...");
    }
    else if (!printerIdle) {
      lastPrinterIdleState = false;  // Reset to detect next idle state
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task 5: Error Handling (Simulate error via button press)
void errorHandlingTask(void *pvParameters) {
  while (true) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      // Simulate printer error
      printerError = true;
      digitalWrite(LED_ERROR_PIN, HIGH);
      tone(BUZZER_PIN, 1000, 500);  // Beep for error
      vTaskDelay(pdMS_TO_TICKS(1000));  // Debounce button
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Periodic check
  }
}
