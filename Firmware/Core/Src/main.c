/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
#include <math.h>
#include "front_calibration.h"
#include "uart_transmit.h"
#include <stdlib.h>
#include "mpu6050.h"
#include "gyro_turn.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAZE_SIZE 16

typedef enum {UP, RIGHT, DOWN, LEFT} Direction;

typedef struct {
    uint8_t up;
    uint8_t right;
    uint8_t down;
    uint8_t left;
} CellWalls;

typedef struct {
    uint8_t distance;   // Flood fill value
    uint8_t visited;
    CellWalls walls;
} Cell;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define DEBOUNCE_TIME 20
#define WINDOW_SIZE 20
/* Global variables to track state and timing */
uint32_t last_led_time = 0;
uint8_t led_state = 0;  // 0 to 5 for the 3 blinks, 6 for the long 3s pause
uint16_t adc_buffer[6];
int16_t Right_encoder_count;
int16_t Left_encoder_count;
int16_t target_speed = 50;
uint32_t target_count;
int16_t speed;

float right_window[WINDOW_SIZE] = {0};
float left_window[WINDOW_SIZE]  = {0};

float right_sum = 0.0f;
float left_sum  = 0.0f;

uint16_t window_index = 0;
uint16_t window_count = 0;
uint8_t search_running = 1;
uint8_t fast_running = 1;
uint8_t continue_flag = 1;
uint8_t side = 1;

volatile uint32_t timerBefore;
volatile uint32_t timerAfter;

float count_per_cell = 1950.0f;

#define FAST_EXTRA_SPEED   10      // PWM added to target_speed on straights (start with 10)
#define FAST_MAX_SPEED     70      // never exceed this
#define FAST_MIN_CELLS     2       // only speed up if at least this many straight cells
#define FAST_DECEL_TICKS   1500    // start slowing this many ticks before the end of the straight
#define FAST_ACCEL_STEP    0.5f    // PWM gained per loop (about 10 ms)
#define FAST_DECEL_STEP    1.5f    // PWM lost per loop

float fast_cur_speed = 50.0f;

// Set when button 2 is pressed during the return-to-start phase of the
// search run. Polled inside the blocking movement/turn routines so a tap
// is caught promptly instead of only being sampled once per cell.
volatile uint8_t stop_requested = 0;

uint8_t Right_barrier_available,Left_barrier_available,Front_barrier_available ;
uint8_t previous_Right_barrier_available,previous_Left_barrier_available ;
uint8_t units ;

uint8_t Right_barrier_available = 1;
uint8_t Left_barrier_available = 1;
uint8_t Front_barrier_available = 0;

uint8_t fast_accel_enabled = 1;

#define PWM_MAX 99
#define PWM_MIN 0

// Flood Fill / Maze Variables
Cell maze[MAZE_SIZE][MAZE_SIZE];
Cell solidmaze[MAZE_SIZE][MAZE_SIZE];
uint8_t robot_x = 0, robot_y = 0;
Direction robot_dir = RIGHT, next_dir = RIGHT;
uint8_t searching_mode = 1; // 1 = Find Center, 2 = Return to Start

typedef struct {
    uint8_t state;       // 0 = idle, 1 = debounce, 2 = pressed
    uint32_t lastTick;
} Button_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float max_integral; // To prevent integral windup
} PID_Controller;

// Initialize PID constants (These will require tuning!)
// 1. Wheel Synchronization PID (Mode 4 / Open Space)
PID_Controller wheel_sync_pid = {
    .Kp = 1.25f,
    .Ki = 0.00035f,
    .Kd = 0.5f,
    .prev_error = 0.0f,
    .integral = 0.0f,
    .max_integral = 50.0f
};

// 2. Spot Turning PID
PID_Controller turn_pid = {
    .Kp = 4.0f,
    .Ki = 0.00f,
    .Kd = 1.50f,
    .prev_error = 0.0f,
    .integral = 0.0f,
    .max_integral = 100.0f
};

// Add this near your other PID_Controller instances (wheel_sync_pid, turn_pid):
PID_Controller wall_square_pid = {
    .Kp = 10.0f,
    .Ki = 0.00f,
    .Kd = 1.0f,
    .prev_error = 0.0f,
    .integral = 0.0f,
    .max_integral = 50.0f
};


typedef struct {
    int16_t right_incl_overshoot;   // was: 100
    int16_t left_incl_overshoot;    // was: 100
    int16_t right_overshoot;        // was: 150
    int16_t left_overshoot;         // was: 150
    int16_t right_bounce;           // was: 400
    int16_t left_bounce;            // was: 400
    int16_t front_bounce;           // was: 10
    int16_t front_appearing_gap;    // was: 200
    int16_t side_disappearing_gap;  // was: 700
    int16_t side_appearing_gap;     // was: 700
    int16_t barrier_unchanged;      // was: 1600
    float   front_contact_distance;
    float mode1_kp, mode1_ki, mode1_kd;   // both walls
    float mode2_kp, mode2_ki, mode2_kd;   // right wall only
    float mode3_kp, mode3_ki, mode3_kd;   // left wall only
} NavThresholds;

static const NavThresholds speed_profiles[7] = {
/*35*/ {100,100,150,150,400,400,10,600,750,700,1750,12.5f,
        4.0f,0.01f,2.0f,  3.5f,0.0001f,2.0f,  3.5f,0.0001f,2.0f},
/*40*/ {75,75,150,150,400,400,10,500,800,700,1750,12.5f,
        1.3f,0.00035f,4.0f,  8.0f,0.00015f,4.0f,  8.0f,0.00015f,4.0f},
/*45*/ {75,75,150,150,450,450,10,550,850,350,1750,12.5f,
        2.0f,0.00035f,5.0f,  8.0f,0.00015f,5.0f,  8.0f,0.00015f,5.0f},
/*50*/ {50,50,175,175,550,550,10,600,800,600,1800,14.5f,
        4.0f,0.000f,2.65f,  8.0f,0.00015f,4.0f,  8.0f,0.00015f,4.0f},
/*60*/ {50,50,175,175,500,500,10,600,800,700,1800,20.0f,
        5.0f,0.00035f,2.5f,  8.0f,0.00015f,4.0f,  8.0f,0.00015f,4.0f},
/*70*/ {50,50,175,175,500,500,10,600,800,700,1800,25.0f,
        8.0f,0.00035f,4.5f,  8.0f,0.00015f,4.0f,  8.0f,0.00015f,4.0f},
};

NavThresholds nav; // active profile, selected at speed-set time
MPU6050_t MPU6050;

Button_t buttons[4] = {0};

GPIO_TypeDef* BUTTON_PORT[4] = {GPIOA, GPIOB, GPIOB, GPIOB};
uint16_t BUTTON_PIN[4] = {GPIO_PIN_10, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void I2C_Scan(I2C_HandleTypeDef *hi2c);
void DISPLAY(char *First, char *Second);
void LED_Loop_NonBlocking(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Motor_Init(void);
void Encoder_Init(void);
float PID_Compute(PID_Controller *pid, float error);
void Right_Motor_Control(int16_t speed);
void MoveForward_Encoder(uint32_t target_count);
void MoveBackward_Encoder(uint32_t target_count);
void Left_Motor_Control(int16_t speed);
void STOP(int16_t target_ticks);
int32_t Get_Left_Encoder(void);
int32_t Get_Right_Encoder(void);
void Reset_Encoders(void);
void TURN_MENU(void);
void TURN_180(void);
void TURN(int16_t target_ticks);
void SPEED_SETTING(void);
void Display_Counter(uint16_t delay);
void Calibrate_At_Wall(void);
void Calibrate_at_left_wall(void);
void Calibrate_at_right_wall(void);
void Print_Front_Averages(void);
void Go_Straight(uint8_t RBA, uint8_t LBA, uint8_t *NRBA, uint8_t *NLBA, uint8_t *NFBA, uint8_t *units);
void go_straight_no_stop(uint8_t RBA, uint8_t LBA, uint8_t *NRBA, uint8_t *NLBA, uint8_t *NFBA, uint8_t *units, uint8_t straight_cells);
void updateFloodFillFastRun(void);
void STOP_MENU(void);
// Flood Fill Prototypes
void InitMaze(void);
void UpdateWalls(uint8_t x, uint8_t y, CellWalls walls);
void UpdateFloodValues(uint8_t goal_x, uint8_t goal_y);
void UpdateFloodValues_2x2Goal(void);
Direction ChooseNextDirection(void);
void TurnRobot(Direction target_dir);
void CopyMaze(Cell dest[MAZE_SIZE][MAZE_SIZE], Cell src[MAZE_SIZE][MAZE_SIZE]);
void getEncoderStatus(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
uint8_t CountStraightCells(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  nav = speed_profiles[3];

  I2C_Scan(&hi2c2);
  uint8_t Button_UpdateMulti(uint8_t idx);
  Motor_Init();
  Encoder_Init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 6);
  SSD1306_Init(&hi2c2);
  while (MPU6050_Init(&hi2c2) == 1);

  HAL_TIM_Base_Start_IT(&htim4);
  Enc_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  DISPLAY("TEAM", "NEURIX");
  uint8_t exit_idle_mode = 0;
  uint8_t gyro_calibrated = 0;


  while (!exit_idle_mode) {

	  LED_Loop_NonBlocking();

      float distance_buffer[6];
      for (int i = 0; i < 6; i++) {
          float voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
          if (voltage > 0.24f)
              distance_buffer[i] = 4.9f / (voltage - 0.075f);
          else
              distance_buffer[i] = 30.0f;
      }

      float Front_distance_1 = distance_buffer[3];
      float Front_distance_2 = distance_buffer[1];
      float Right_distance = distance_buffer[0];
      float Left_distance = distance_buffer[5];
      float Right_inclined_distance = distance_buffer[2];
      float Left_inclined_distance = distance_buffer[4];

      float Front_distance = (Front_distance_1 + Front_distance_2) / 2.0f;

      printf("L: %.1f | LI: %.1f | F1: %.1f | F2: %.1f | F_avg: %.1f | RI: %.1f | R: %.1f cm\r\n",
             Left_distance,
             Left_inclined_distance,
             Front_distance_1,
             Front_distance_2,
             Front_distance,
             Right_inclined_distance,
             Right_distance);

      for (uint8_t i = 0; i < 4; i++) {
          if (Button_UpdateMulti(i)) {
              if (i == 0) {
                  SPEED_SETTING();
              }
              if (i == 1) {
            	  fast_accel_enabled = !fast_accel_enabled;
            	  if (fast_accel_enabled) {
            	      DISPLAY("ACCEL", "ON");
            	  } else {
            	      DISPLAY("ACCEL", "OFF");
            	  }
            	  HAL_Delay(500);
            	  DISPLAY("TEAM", "NEURIX");
              }
              if (i == 2) {
            	  if(gyro_calibrated==1){
                      DISPLAY("SEARCH","RUN");
                      Display_Counter(500);
                      exit_idle_mode = 1;
            	  }else{
            		  DISPLAY("CALIB.","GYRO");
                	  HAL_Delay(500);
                	  DISPLAY("TEAM", "NEURIX");
            	  }

              }
              if (i == 3) {
            	  DISPLAY("GYRO", "CALIBRATING");
            	  HAL_Delay(500);

            	  if (Gyro_Calibrate() != 0)
            	  {
            	      DISPLAY("CAL", "FAILED");
            	      while (1);
            	  }

            	  printf("Gz offset = %.3f dps\r\n", Gyro_Get_Z_Offset());
            	  gyro_calibrated = 1;
            	  DISPLAY("GYRO", "READY");
            	  HAL_Delay(500);
            	  DISPLAY("TEAM", "NEURIX");
              }
          }
      }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  robot_x = 0;
  robot_y = 0;
  robot_dir = RIGHT;
  searching_mode = 1;
  search_running = 1;
  stop_requested = 0;
  uint8_t first = 1;

  InitMaze();
  CopyMaze(maze, solidmaze);

  while (search_running == 1)
  {
	  if(searching_mode==1){
		  DISPLAY("SEARCHING","ROUND");
	  }
	  LED_Loop_NonBlocking();

      if (searching_mode == 2 && stop_requested) {
          DISPLAY("MANUAL","STOP!");
          HAL_Delay(1000);
          search_running = 0; // Forcefully end the search run
          stop_requested = 0;
          break;
      }

	  previous_Right_barrier_available = Right_barrier_available;
	  previous_Left_barrier_available = Left_barrier_available;

	  Go_Straight(previous_Right_barrier_available,
			  previous_Left_barrier_available,
			  &Right_barrier_available,
			  &Left_barrier_available,
			  &Front_barrier_available,
			  &units);

	  if (searching_mode == 2 && stop_requested) {
	          DISPLAY("MANUAL","STOP!");
	          HAL_Delay(1000);
	          search_running = 0;
	          stop_requested = 0;
	          break;
	  }

	  if (first == 1) {
		  if (Right_barrier_available == 1){
			  CellWalls walls_first = {1, 0, 1, 1};
			  UpdateWalls(0, 0, walls_first);
			  robot_dir = RIGHT;
		  }
		  else if (Left_barrier_available == 1){
			  CellWalls walls_first = {0, 1, 1, 1};
			  UpdateWalls(0, 0, walls_first);
			  robot_dir = UP;
		  }

		  maze[0][0].visited = 1;
		  first = 0;
	  }

	  CellWalls horizontal_walls = {1, 0, 1, 0};
	  CellWalls vertical_walls = {0, 1, 0, 1};
	  CellWalls present_cell;

	  if (robot_dir == UP){
		  for (int i = 1; i < units; i++){
			   UpdateWalls(robot_x, robot_y + i, vertical_walls);
			   maze[robot_x][robot_y + i].visited = 1;
		  }
		  robot_y = robot_y + units;


		  present_cell.up = Front_barrier_available;
		  present_cell.down = 0;
		  present_cell.right = Right_barrier_available;
		  present_cell.left = Left_barrier_available;
	  }
	  else if (robot_dir == DOWN){
		  for (int i = 1; i < units; i++){
			   UpdateWalls(robot_x, robot_y - i, vertical_walls);
			   maze[robot_x][robot_y - i].visited = 1;
		  }
		  robot_y = robot_y - units;
		  present_cell.up = 0;
		  present_cell.down = Front_barrier_available;
		  present_cell.right = Left_barrier_available;
		  present_cell.left = Right_barrier_available;
	  }
	  else if (robot_dir == RIGHT){
		  for (int i = 1; i < units; i++){
			   UpdateWalls(robot_x + i, robot_y, horizontal_walls);
			   maze[robot_x + i][robot_y].visited = 1;
		  }
		  robot_x = robot_x + units;
		  present_cell.up = Left_barrier_available;
		  present_cell.down = Right_barrier_available;
		  present_cell.right = Front_barrier_available;
		  present_cell.left = 0;
	  }
	  else if (robot_dir == LEFT){
		  for (int i = 1; i < units; i++){
			   UpdateWalls(robot_x - i, robot_y, horizontal_walls);
			   maze[robot_x - i][robot_y].visited = 1;
		  }
		  robot_x = robot_x - units;
		  present_cell.up = Right_barrier_available;
		  present_cell.down = Left_barrier_available;
		  present_cell.right = 0;
		  present_cell.left = Front_barrier_available;
	  }

	  UpdateWalls(robot_x, robot_y, present_cell);
	  maze[robot_x][robot_y].visited = 1;

	  if (((robot_x == 7) || (robot_x == 8)) && ((robot_y == 7) || (robot_y == 8))){
		  CopyMaze(solidmaze, maze);
		  searching_mode = 2;

		  DISPLAY("PATH","FOUND");
		  for(int i=0 ; i<5 ; i++){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			  HAL_Delay(100);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_Delay(100);

		  }
		  DISPLAY("RETURNING","BACK");
	  }
	  if ((robot_x == 0) && (robot_y == 0) && (searching_mode == 2)){
		  search_running = 0;

		  for(int i=0 ; i<5 ; i++){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			  HAL_Delay(100);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_Delay(100);

		  }
	  }

	  if (searching_mode == 1){
		  UpdateFloodValues_2x2Goal();
	  }
	  else {
		  UpdateFloodValues(0, 0);
	  }

	  next_dir = ChooseNextDirection();
	  TurnRobot(next_dir);
	  robot_dir = next_dir;
  }

while(1)
  {
      DISPLAY("DONE","SEARCH");
      LED_Loop_NonBlocking();

      if(Button_UpdateMulti(0)){
    	  SPEED_SETTING();
      }

      if (Button_UpdateMulti(1)) {
          fast_accel_enabled = !fast_accel_enabled;
          if (fast_accel_enabled) {
              DISPLAY("ACCEL", "ON");
          } else {
              DISPLAY("ACCEL", "OFF");
          }
          HAL_Delay(500);
          DISPLAY("TEAM", "NEURIX");
      }
      // First press of Button 3: Ask for confirmation
      if (Button_UpdateMulti(3))
      {
          DISPLAY("FAST RUN", "CONFIRM?");
          HAL_Delay(300); // Small guard delay

          uint8_t confirmed = 0;
          uint8_t waiting_for_confirmation = 1;

          // Confirmation loop
          while (waiting_for_confirmation)
          {
              LED_Loop_NonBlocking();

              // If Button 3 is pressed again -> CONFIRMED
              if (Button_UpdateMulti(3))
              {
                  confirmed = 1;
                  waiting_for_confirmation = 0;
              }
              // If any other button (0, 1, or 2) is pressed -> CANCEL
              else if (Button_UpdateMulti(0) || Button_UpdateMulti(1) || Button_UpdateMulti(2))
              {
                  confirmed = 0;
                  waiting_for_confirmation = 0;
              }
          }

          // If confirmed, execute Fast Run
          if (confirmed)
          {
              // --- FAST RUN EXECUTION ---
              DISPLAY("FAST","RUN");
              Display_Counter(500);

              uint8_t first = 1;
              Right_barrier_available = 1;
              Left_barrier_available = 1;
              robot_x = 0;
              robot_y = 0;
              stop_requested = 0; // clear any stale manual-stop flag before starting
              CopyMaze(maze, solidmaze);
      	      maze[7][7].visited = 1;
      	      maze[7][8].visited = 1;
      	      maze[8][7].visited = 1;
      	      maze[8][8].visited = 1;
              updateFloodFillFastRun();
              fast_cur_speed = target_speed;

              uint8_t fast_running = 1;
              while(fast_running)
              {
                  previous_Right_barrier_available = Right_barrier_available;
                  previous_Left_barrier_available = Left_barrier_available;

                  uint8_t straight_cells = first ? 0 : CountStraightCells();

                  go_straight_no_stop(previous_Right_barrier_available,
                                      previous_Left_barrier_available,
                                      &Right_barrier_available,
                                      &Left_barrier_available,
                                      &Front_barrier_available,
                                      &units,
                                      straight_cells);

                  if (stop_requested)
                  {
                      DISPLAY("MANUAL","STOP!");
                      HAL_Delay(1000);
                      STOP(75);
                      fast_running = 0;
                      stop_requested = 0;
                      break;
                  }

                  if (first == 1) {
                       if (Right_barrier_available == 1){
                           robot_dir = RIGHT;
                       }
                       else if (Left_barrier_available == 1){
                           robot_dir = UP;
                       }
                       first = 0;
                  }

                  if (robot_dir == UP){
                      robot_y = robot_y + units;
                  }
                  else if (robot_dir == DOWN){
                      robot_y = robot_y - units;
                  }
                  else if (robot_dir == RIGHT){
                      robot_x = robot_x + units;
                  }
                  else if (robot_dir == LEFT){
                      robot_x = robot_x - units;
                  }

                  if (((robot_x == 7) || (robot_x == 8)) && ((robot_y == 7) || (robot_y == 8))){
                      STOP(75);
                      fast_running = 0;
                  }

                  next_dir = ChooseNextDirection();
                  if (!(robot_dir == next_dir)){

                	  fast_cur_speed = target_speed;

                    	if(Front_barrier_available){
                    		Calibrate_At_Wall();
                    	}else{
                    		STOP(75);
                    	}
                  }
                  TurnRobot(next_dir);
                  robot_dir = next_dir;
              }

			  DISPLAY("DONE","FAST RUN");
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			  HAL_Delay(100);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_Delay(100);
			  HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);

          }
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 95;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 STBY_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_7|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


uint8_t Button_UpdateMulti(uint8_t idx)
{
    // read the button state
	uint8_t pressed;
	if (HAL_GPIO_ReadPin(BUTTON_PORT[idx], BUTTON_PIN[idx]) == GPIO_PIN_RESET){
		pressed = 1;
	}
	else{
		pressed = 0;
	}
    uint32_t now = HAL_GetTick();

    // IDLE
    if (buttons[idx].state == 0)
    {
        if (pressed)
        {
            buttons[idx].state = 1;
            buttons[idx].lastTick = now;
        }
    }

    // DEBOUNCE
    if (buttons[idx].state == 1)
    {
        if ((now - buttons[idx].lastTick) >= DEBOUNCE_TIME)
        {
            if (pressed)
            {
                buttons[idx].state = 2;
                return 1;  // one press detected
            }
            else
            {
                buttons[idx].state = 0;  // false trigger
            }
        }
    }

    // PRESSED
    if (buttons[idx].state == 2)
    {
        if (!pressed)
        {
            buttons[idx].state = 0;  // released
        }
    }

    return 0;
}

int32_t Get_Right_Encoder(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

int32_t Get_Left_Encoder(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

void Reset_Encoders(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(GPIOA, STBY_Pin, GPIO_PIN_SET);
}

void Encoder_Init(void)
{
    /* Start both encoder timers */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* Reset counters to zero */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}


void Left_Motor_Control(int16_t speed)
{
    uint16_t pwm = abs(speed);

    if(speed > 0)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else if(speed < 0)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    else
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);

}

void Right_Motor_Control(int16_t speed)
{
	uint16_t pwm = abs(speed);

    if(speed > 0)
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
    else if(speed < 0)
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
}

void MoveForward_Encoder(uint32_t target_count)
{
    int32_t avg = 0;

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    wheel_sync_pid.integral = 0;
    wheel_sync_pid.prev_error = 0;

    while (1)
    {
        Right_encoder_count = Get_Right_Encoder();
        Left_encoder_count = Get_Left_Encoder();

        avg = (Right_encoder_count + Left_encoder_count) / 2;

        int16_t error = Left_encoder_count - Right_encoder_count;

        float correction = PID_Compute(&wheel_sync_pid, (float)error);

        int16_t left_pwm  = (int16_t)(target_speed) - (int16_t)correction;
        int16_t right_pwm = (int16_t)(target_speed) + (int16_t)correction;

        Left_Motor_Control(left_pwm);
        Right_Motor_Control(right_pwm);

        avg = (Right_encoder_count + Left_encoder_count) / 2;
        if (avg >= target_count)
        {
            break;
        }

        HAL_Delay(10);
    }

    Left_Motor_Control(0);
    Right_Motor_Control(0);
}

void MoveBackward_Encoder(uint32_t target_count)
{
    int32_t avg = 0;

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    wheel_sync_pid.integral = 0;
    wheel_sync_pid.prev_error = 0;

    while (1)
    {
        Right_encoder_count = Get_Right_Encoder();
        Left_encoder_count = Get_Left_Encoder();

        avg = (Right_encoder_count + Left_encoder_count) / 2;

        int16_t error = Left_encoder_count - Right_encoder_count;
        float correction = PID_Compute(&wheel_sync_pid, (float)error);

        int16_t left_pwm  = (int16_t)(-target_speed) - (int16_t)correction;
        int16_t right_pwm = (int16_t)(-target_speed) + (int16_t)correction;

        if(left_pwm > 0) left_pwm = 0;
        if(left_pwm < -(target_speed)) left_pwm = -(target_speed);
        if(right_pwm > 0) right_pwm = 0;
        if(right_pwm < -(target_speed)) right_pwm = -(target_speed);

        Left_Motor_Control(left_pwm);
        Right_Motor_Control(right_pwm);

        if (avg <= -(int32_t)target_count)
        {
            break;
        }

        HAL_Delay(10);
    }

    Left_Motor_Control(0);
    Right_Motor_Control(0);
}

// Flood Fill Helper Functions
void InitMaze(void)
{
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            maze[x][y].walls.up = 0;
            maze[x][y].walls.right = 0;
            maze[x][y].walls.down = 0;
            maze[x][y].walls.left = 0;
            maze[x][y].visited = 0;
            maze[x][y].distance = 255;

            if (y == MAZE_SIZE - 1) maze[x][y].walls.up = 1;
            if (y == 0)             maze[x][y].walls.down = 1;
            if (x == MAZE_SIZE - 1) maze[x][y].walls.right = 1;
            if (x == 0)             maze[x][y].walls.left = 1;
        }
    }
}

void UpdateWalls(uint8_t x, uint8_t y, CellWalls walls) {
    maze[x][y].walls = walls;

    if (x < MAZE_SIZE - 1) maze[x + 1][y].walls.left = walls.right;
    if (x > 0)             maze[x - 1][y].walls.right = walls.left;
    if (y < MAZE_SIZE - 1) maze[x][y + 1].walls.down = walls.up;
    if (y > 0)             maze[x][y - 1].walls.up = walls.down;
}

void UpdateFloodValues(uint8_t goal_x, uint8_t goal_y) {
    for (int i = 0; i < MAZE_SIZE; i++)
        for (int j = 0; j < MAZE_SIZE; j++)
            maze[i][j].distance = 255;

    maze[goal_x][goal_y].distance = 0;
    uint8_t changed = 1;

    while (changed) {
        changed = 0;
        for (int x = 0; x < MAZE_SIZE; x++) {
            for (int y = 0; y < MAZE_SIZE; y++) {
                CellWalls w = maze[x][y].walls;
                uint8_t d = maze[x][y].distance;

                if (!w.up && y < MAZE_SIZE - 1 && d + 1 < maze[x][y + 1].distance) {
                    maze[x][y + 1].distance = d + 1; changed = 1;
                }
                if (!w.down && y > 0 && d + 1 < maze[x][y - 1].distance) {
                    maze[x][y - 1].distance = d + 1; changed = 1;
                }
                if (!w.right && x < MAZE_SIZE - 1 && d + 1 < maze[x + 1][y].distance) {
                    maze[x + 1][y].distance = d + 1; changed = 1;
                }
                if (!w.left && x > 0 && d + 1 < maze[x - 1][y].distance) {
                    maze[x - 1][y].distance = d + 1; changed = 1;
                }
            }
        }
    }
}

void UpdateFloodValues_2x2Goal(void)
{
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            maze[x][y].distance = 255;

    uint8_t cx1 = MAZE_SIZE / 2 - 1;
    uint8_t cx2 = MAZE_SIZE / 2;
    uint8_t cy1 = MAZE_SIZE / 2 - 1;
    uint8_t cy2 = MAZE_SIZE / 2;

    maze[cx1][cy1].distance = 0;
    maze[cx1][cy2].distance = 0;
    maze[cx2][cy1].distance = 0;
    maze[cx2][cy2].distance = 0;

    uint8_t changed = 1;
    while (changed) {
        changed = 0;
        for (int x = 0; x < MAZE_SIZE; x++) {
            for (int y = 0; y < MAZE_SIZE; y++) {
                CellWalls w = maze[x][y].walls;
                uint8_t d = maze[x][y].distance;

                if (d == 255) continue;

                if (!w.up && y < MAZE_SIZE - 1 && d + 1 < maze[x][y + 1].distance) {
                    maze[x][y + 1].distance = d + 1; changed = 1;
                }
                if (!w.down && y > 0 && d + 1 < maze[x][y - 1].distance) {
                    maze[x][y - 1].distance = d + 1; changed = 1;
                }
                if (!w.right && x < MAZE_SIZE - 1 && d + 1 < maze[x + 1][y].distance) {
                    maze[x + 1][y].distance = d + 1; changed = 1;
                }
                if (!w.left && x > 0 && d + 1 < maze[x - 1][y].distance) {
                    maze[x - 1][y].distance = d + 1; changed = 1;
                }
            }
        }
    }
}

Direction ChooseNextDirection(void) {
    Cell current = maze[robot_x][robot_y];
    Direction best_dir = robot_dir;
    uint8_t min_distance = 255;


    if (!current.walls.up && robot_y < MAZE_SIZE - 1 && maze[robot_x][robot_y + 1].distance < min_distance) {
        min_distance = maze[robot_x][robot_y + 1].distance;
        best_dir = UP;
    }
    if (!current.walls.down && robot_y > 0 && maze[robot_x][robot_y - 1].distance < min_distance) {
        min_distance = maze[robot_x][robot_y - 1].distance;
        best_dir = DOWN;
    }

    if (!current.walls.right && robot_x < MAZE_SIZE - 1 && maze[robot_x + 1][robot_y].distance < min_distance) {
        min_distance = maze[robot_x + 1][robot_y].distance;
        best_dir = RIGHT;
    }
    if (!current.walls.left && robot_x > 0 && maze[robot_x - 1][robot_y].distance < min_distance) {
        min_distance = maze[robot_x - 1][robot_y].distance;
        best_dir = LEFT;
    }

    return best_dir;
}

void TurnRobot(Direction target_dir)
{
    int8_t diff = target_dir - robot_dir;

    // Normalize to range -2 to +2
    if (diff > 2)  diff -= 4;
    if (diff < -2) diff += 4;

    uint8_t RB = Right_barrier_available;
    uint8_t LB = Left_barrier_available;
    uint8_t FB = Front_barrier_available;

    switch (diff)
    {
        case 0:{
            // No turn
            break;
        }
        case 1:{
            // Turn right 90°
        	Right_barrier_available = 0;
        	Left_barrier_available = FB;
        	TURN_GYRO(-90);
            break;
        }
        case -1:{
            // Turn left 90°
        	Right_barrier_available = FB;
        	Left_barrier_available = 0;
        	TURN_GYRO(90);
            break;
        }
        case 2:
        case -2:{
            // Turn 180°
        	Right_barrier_available = LB;
        	Left_barrier_available = RB;
        	TURN_180();
            break;
        }
    }


}

void CopyMaze(Cell dest[MAZE_SIZE][MAZE_SIZE], Cell src[MAZE_SIZE][MAZE_SIZE])
{
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            dest[y][x] = src[y][x];
        }
    }
}

/* ---- Gyro straight-line settings ---- */
#define STRAIGHT_KP        3.0f    // PWM per degree of heading error
#define STRAIGHT_KD        0.15f   // PWM per deg/s (damping)
#define STRAIGHT_KI        0.0f
#define STRAIGHT_MAX_CORR  20.0f   // max PWM added/removed per wheel by the gyro
#define RATE_FILT_ALPHA    0.2f
#define GYRO_SIGN          1.0f    // must match the value you use in gyro_turn.c
#define GYRO_SCALE         1.0f

void Go_Straight(uint8_t RBA, uint8_t LBA, uint8_t *NRBA, uint8_t *NLBA, uint8_t *NFBA, uint8_t *units)
{
    int16_t Right_inclined_overshoot_count = 0, Left_inclined_overshoot_count = 0;
    int16_t Right_overshoot_count = 0, Left_overshoot_count = 0;
    int16_t Right_bounce_count = 0, Left_bounce_count = 0, Front_bounce_count = 0;

    uint8_t mode = 4;
    float spot_from_right = 0.0f, spot_from_left = 0.0f, spot_from_middle = 0.0f;
    uint8_t Right_barrier_available = RBA, Left_barrier_available = LBA, Front_barrier_available = 0;
    uint8_t Right_levelling = RBA, Left_levelling = LBA;

    uint8_t running = 1;
    uint8_t levelling_changed = 1, barrier_changed = 0;

    uint8_t front_barrier_appearing = 0;
    uint8_t side_barrier_appearing = 0;
    uint8_t side_barrier_disappearing = 0;
    uint8_t barrier_unchanging = 1;
    uint8_t to_take_decisions = 0;

    int16_t front_barrier_appearing_count = 0;
    int16_t side_barrier_appearing_count = 0;
    int16_t side_barrier_disappearing_count = 0;
    int16_t barrier_unchanged_count = nav.barrier_unchanged;

    int base_speed = target_speed;
    float danger_speed = 15;

    /* encoder counts (declared here so they can be used after the loop) */
    int16_t Right_encoder_count = 0, Left_encoder_count = 0;

    /* gyro state */
    const float gyro_z_offset = Gyro_Get_Z_Offset();
    float yaw = 0.0f, yaw_integral = 0.0f, rate_filt = 0.0f;
    uint8_t rate_init = 0;
    uint32_t now;

    Reset_Encoders();

    float distance_buffer[6];
    for (int i = 0; i < 6; i++) {
        float voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
        if (voltage > 0.24f)
            distance_buffer[i] = 4.9f / (voltage - 0.075f);
        else
            distance_buffer[i] = 30.0f;
    }

    float Front_distance_1 = distance_buffer[3];
    float Front_distance_2 = distance_buffer[1];
    float Right_distance = distance_buffer[0];
    float Left_distance = distance_buffer[5];
    float Right_inclined_distance = distance_buffer[2];
    float Left_inclined_distance = distance_buffer[4];
    float Front_distance = (Front_distance_1 + Front_distance_2) / 2.0f;

    if (Right_inclined_distance > 12.5f) Right_levelling = 0;
    if (Left_inclined_distance > 12.5f)  Left_levelling = 0;

    float error = 0, last_error = 0, integral = 0, derivative = 0;
    uint32_t start = HAL_GetTick();
    uint32_t last  = start;

    while (running && (continue_flag == 1))
    {
        LED_Loop_NonBlocking();

        if (searching_mode == 2 && Button_UpdateMulti(2)) {
            stop_requested = 1;
            Left_Motor_Control(0);
            Right_Motor_Control(0);
            break;
        }

        for (int i = 0; i < 6; i++) {
            float voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
            if (voltage > 0.24f)
                distance_buffer[i] = 4.9f / (voltage - 0.075f);
            else
                distance_buffer[i] = 30.0f;
        }

        Front_distance_1 = distance_buffer[3];
        Front_distance_2 = distance_buffer[1];
        Right_distance = distance_buffer[0];
        Left_distance = distance_buffer[5];
        Right_inclined_distance = distance_buffer[2];
        Left_inclined_distance = distance_buffer[4];
        Front_distance = (Front_distance_1 + Front_distance_2) / 2.0f;

        Right_encoder_count = Get_Right_Encoder();
        Left_encoder_count  = Get_Left_Encoder();

        if (Right_levelling == 1) {
            if (Right_inclined_distance > 20.0f) {
                if (Right_encoder_count > Right_inclined_overshoot_count) {
                    Right_levelling = 0;
                    levelling_changed = 1;
                }
            } else {
                Right_inclined_overshoot_count = nav.right_incl_overshoot + Right_encoder_count;
            }
        }

        if (Left_levelling == 1) {
            if (Left_inclined_distance > 20.0f) {
                if (Left_encoder_count > Left_inclined_overshoot_count) {
                    Left_levelling = 0;
                    levelling_changed = 1;
                }
            } else {
                Left_inclined_overshoot_count = nav.left_incl_overshoot + Left_encoder_count;
            }
        }

        if (Front_distance < nav.front_contact_distance) {
            if (Front_barrier_available == 0) {
                if (((Right_encoder_count + Left_encoder_count) / 2) > Front_bounce_count) {
                    Front_barrier_available = 1;
                    front_barrier_appearing = 1;
                    front_barrier_appearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.front_appearing_gap;
                }
            }
        } else {
            Front_bounce_count = nav.front_bounce + ((Right_encoder_count + Left_encoder_count) / 2);
        }

        if (Right_distance > 20.0f) {
            if (Right_barrier_available == 1) {
                if (Right_encoder_count > Right_overshoot_count) {
                    Right_barrier_available = 0;
                    barrier_changed = 1;
                    side_barrier_disappearing = 1;
                    side_barrier_disappearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_disappearing_gap;
                }
            } else {
                Right_bounce_count = nav.right_bounce + Right_encoder_count;
            }
        } else {
            if (Right_barrier_available == 1) {
                Right_overshoot_count = nav.right_overshoot + Right_encoder_count;
            } else {
                if (Right_encoder_count > Right_bounce_count) {
                    Right_barrier_available = 1;
                    barrier_changed = 1;
                    side_barrier_appearing = 1;
                    side_barrier_appearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_appearing_gap;
                    Right_levelling = 1;
                    levelling_changed = 1;
                }
            }
        }

        if (Left_distance > 20.0f) {
            if (Left_barrier_available == 1) {
                if (Left_encoder_count > Left_overshoot_count) {
                    Left_barrier_available = 0;
                    side_barrier_disappearing = 1;
                    side_barrier_disappearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_disappearing_gap;
                }
            } else {
                Left_bounce_count = nav.left_bounce + Left_encoder_count;
            }
        } else {
            if (Left_barrier_available == 1) {
                Left_overshoot_count = nav.left_overshoot + Left_encoder_count;
            } else {
                if (Left_encoder_count > Left_bounce_count) {
                    Left_barrier_available = 1;
                    side_barrier_appearing = 1;
                    side_barrier_appearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_appearing_gap;
                    Left_levelling = 1;
                    levelling_changed = 1;
                }
            }
        }

        spot_from_middle = 0;
        if (levelling_changed == 1)
        {
            if ((Right_levelling == 1) && (Left_levelling == 1)) {
                mode = 1;
            } else if ((Right_levelling == 1) && (Left_levelling == 0)) {
                mode = 2;
                spot_from_right = Right_distance;
            } else if ((Right_levelling == 0) && (Left_levelling == 1)) {
                mode = 3;
                spot_from_left = Left_distance;
            } else {
                mode = 4;   // no walls: gyro-only straight
            }

            /* reset wall PID */
            integral = 0;
            derivative = 0;
            last_error = 0;

            /* reset gyro heading reference for the new mode */
            yaw = 0.0f;
            yaw_integral = 0.0f;

            levelling_changed = 0;
        }

        if (front_barrier_appearing == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > front_barrier_appearing_count) {
                to_take_decisions = 1;
            }
        } else if (side_barrier_disappearing == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > side_barrier_disappearing_count) {
                to_take_decisions = 1;
            }
        } else if (side_barrier_appearing == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > side_barrier_appearing_count) {
                to_take_decisions = 1;
            }
        } else if (barrier_unchanging == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > barrier_unchanged_count) {
                to_take_decisions = 1;
            }
        }

        if (to_take_decisions == 1) {
            if ((Right_barrier_available == 1) && (Left_barrier_available == 1) && (Front_barrier_available == 0)) {
                front_barrier_appearing = 0;
                side_barrier_appearing = 0;
                side_barrier_disappearing = 0;
                barrier_unchanging = 0;
                to_take_decisions = 0;
            } else {
                running = 0;
                if (Front_barrier_available) {
                    Calibrate_At_Wall();

                } else {
                    STOP(75);
                }
                continue;
            }
        }

        /* ---------------- wall PID ---------------- */
        float output;

        if (mode == 1) {
            float Kp = nav.mode1_kp;
            float Ki = nav.mode1_ki;
            float Kd = nav.mode1_kd;

            error = Left_distance - Right_distance - spot_from_middle;
            if (fabsf(error) < 0.5f)
            {
                error = 0;
            }
            integral += error;
            derivative = error - last_error;
            output = Kp * error + Ki * integral + Kd * derivative;
            last_error = error;
        }
        else if (mode == 2) {
            float Kp = nav.mode2_kp;
            float Ki = nav.mode2_ki;
            float Kd = nav.mode2_kd;

            error = spot_from_right - Right_distance;

            integral += error;
            derivative = error - last_error;
            output = Kp * error + Ki * integral + Kd * derivative;
            last_error = error;
        }
        else if (mode == 3) {
            float Kp = nav.mode3_kp;
            float Ki = nav.mode3_ki;
            float Kd = nav.mode3_kd;

            error = Left_distance - spot_from_left;

            integral += error;
            derivative = error - last_error;
            output = Kp * error + Ki * integral + Kd * derivative;
            last_error = error;
        }
        else {
            output = 0;
        }

        /* ---------------- gyro ---------------- */
        if (MPU6050_Read_Gyro(&hi2c2, &MPU6050) != HAL_OK)
        {
            HAL_Delay(1);
            continue;
        }

        now = HAL_GetTick();
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        if (dt <= 0.0f)
            continue;

        float rate = GYRO_SIGN * ((float)MPU6050.Gz - gyro_z_offset);

        if (!rate_init) {
            rate_filt = rate;
            rate_init = 1;
        } else {
            rate_filt = (1.0f - RATE_FILT_ALPHA) * rate_filt + RATE_FILT_ALPHA * rate;
        }

        yaw += rate * dt * GYRO_SCALE;      // + = drifted left (CCW)

        yaw_integral += yaw * dt;
        if (yaw_integral >  50.0f) yaw_integral =  50.0f;
        if (yaw_integral < -50.0f) yaw_integral = -50.0f;

        float corr;
        if (mode == 4) {
            /* no walls: full gyro heading hold */
            corr = STRAIGHT_KP * yaw + STRAIGHT_KI * yaw_integral + STRAIGHT_KD * rate_filt;
        } else {
            /* walls steer the robot, gyro only damps wobble */
            corr = STRAIGHT_KD * rate_filt;
        }

        if (corr >  STRAIGHT_MAX_CORR) corr =  STRAIGHT_MAX_CORR;
        if (corr < -STRAIGHT_MAX_CORR) corr = -STRAIGHT_MAX_CORR;

        /* ---------------- motors ---------------- */

        float l, r;

        if ((fabsf(output) > 10.0f)||(Right_inclined_distance < 3.0)||(Left_inclined_distance < 3.0)) {
            l = (float)danger_speed - output;
            r = (float)danger_speed + output;
        } else {
            l = (float)base_speed - 0.5f * output + corr;
            r = (float)base_speed + 0.5f * output - corr;
        }

        if (l > PWM_MAX) l = PWM_MAX;
        if (l < PWM_MIN) l = PWM_MIN;
        if (r > PWM_MAX) r = PWM_MAX;
        if (r < PWM_MIN) r = PWM_MIN;

        Left_Motor_Control((int16_t)l);
        Right_Motor_Control((int16_t)r);

        HAL_Delay(8);
    }

    Right_encoder_count = Get_Right_Encoder();
    Left_encoder_count  = Get_Left_Encoder();
    int16_t total_count = (Right_encoder_count + Left_encoder_count) / 2;
    float count = total_count / count_per_cell;
    int r = (int)(count + 0.5f);

    *NRBA = Right_barrier_available;
    *NLBA = Left_barrier_available;
    *NFBA = Front_barrier_available;
    *units = (uint8_t)r;
}



/* Same logic as ChooseNextDirection, but for any cell */
static Direction ChooseDirAt(uint8_t x, uint8_t y, Direction cur)
{
    Cell c = maze[x][y];
    Direction best = cur;
    uint8_t min_d = 255;

    if (!c.walls.up && y < MAZE_SIZE - 1 && maze[x][y + 1].distance < min_d) {
        min_d = maze[x][y + 1].distance; best = UP;
    }
    if (!c.walls.down && y > 0 && maze[x][y - 1].distance < min_d) {
        min_d = maze[x][y - 1].distance; best = DOWN;
    }
    if (!c.walls.right && x < MAZE_SIZE - 1 && maze[x + 1][y].distance < min_d) {
        min_d = maze[x + 1][y].distance; best = RIGHT;
    }
    if (!c.walls.left && x > 0 && maze[x - 1][y].distance < min_d) {
        min_d = maze[x - 1][y].distance; best = LEFT;
    }
    return best;
}

/* How many cells the planned path continues straight from the current cell */
uint8_t CountStraightCells(void)
{
    uint8_t x = robot_x, y = robot_y;
    uint8_t count = 0;

    while (count < MAZE_SIZE)
    {
        if (maze[x][y].distance == 0 || maze[x][y].distance == 255) break;
        if (ChooseDirAt(x, y, robot_dir) != robot_dir) break;

        if      (robot_dir == UP)    { if (y >= MAZE_SIZE - 1) break; y++; }
        else if (robot_dir == DOWN)  { if (y == 0) break;             y--; }
        else if (robot_dir == RIGHT) { if (x >= MAZE_SIZE - 1) break; x++; }
        else                         { if (x == 0) break;             x--; }
        count++;
    }
    return count;
}

void go_straight_no_stop(uint8_t RBA, uint8_t LBA, uint8_t *NRBA, uint8_t *NLBA, uint8_t *NFBA, uint8_t *units, uint8_t straight_cells)
{
    int16_t Right_inclined_overshoot_count = 0, Left_inclined_overshoot_count = 0;
    int16_t Right_overshoot_count = 0, Left_overshoot_count = 0;
    int16_t Right_bounce_count = 0, Left_bounce_count = 0, Front_bounce_count = 0;

    uint8_t mode = 4;
    float spot_from_right = 0.0f, spot_from_left = 0.0f, spot_from_middle = 0.0f;
    uint8_t Right_barrier_available = RBA, Left_barrier_available = LBA, Front_barrier_available = 0;
    uint8_t Right_levelling = RBA, Left_levelling = LBA;

    uint8_t running = 1;
    uint8_t levelling_changed = 1, barrier_changed = 0;

    uint8_t front_barrier_appearing = 0;
    uint8_t side_barrier_appearing = 0;
    uint8_t side_barrier_disappearing = 0;
    uint8_t barrier_unchanging = 1;
    uint8_t to_take_decisions = 0;

    int16_t front_barrier_appearing_count = 0;
    int16_t side_barrier_appearing_count = 0;
    int16_t side_barrier_disappearing_count = 0;
    int16_t barrier_unchanged_count = nav.barrier_unchanged;

    int base_speed = target_speed;
    float danger_speed = 15;
    int16_t Right_encoder_count = 0, Left_encoder_count = 0;

    /* gyro state */
    const float gyro_z_offset = Gyro_Get_Z_Offset();
    float yaw = 0.0f, yaw_integral = 0.0f, rate_filt = 0.0f;
    uint8_t rate_init = 0;
    uint32_t now;

    Reset_Encoders();

    float distance_buffer[6];
    for (int i = 0; i < 6; i++) {
        float voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
        if (voltage > 0.24f)
            distance_buffer[i] = 4.9f / (voltage - 0.075f);
        else
            distance_buffer[i] = 30.0f;
    }

    float Front_distance_1 = distance_buffer[3];
    float Front_distance_2 = distance_buffer[1];
    float Right_distance = distance_buffer[0];
    float Left_distance = distance_buffer[5];
    float Right_inclined_distance = distance_buffer[2];
    float Left_inclined_distance = distance_buffer[4];
    float Front_distance = (Front_distance_1 + Front_distance_2) / 2.0f;

    if (Right_inclined_distance > 12.5f) Right_levelling = 0;
    if (Left_inclined_distance > 12.5f)  Left_levelling = 0;

    float error = 0, last_error = 0, integral = 0, derivative = 0;
    uint32_t start = HAL_GetTick();
    uint32_t last  = start;

    while (running && (continue_flag == 1))
    {
        LED_Loop_NonBlocking();

        if (searching_mode == 2 && Button_UpdateMulti(2)) {
            stop_requested = 1;
            Left_Motor_Control(0);
            Right_Motor_Control(0);
            break;
        }

        for (int i = 0; i < 6; i++) {
            float voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
            if (voltage > 0.24f)
                distance_buffer[i] = 4.9f / (voltage - 0.075f);
            else
                distance_buffer[i] = 30.0f;
        }

        Front_distance_1 = distance_buffer[3];
        Front_distance_2 = distance_buffer[1];
        Right_distance = distance_buffer[0];
        Left_distance = distance_buffer[5];
        Right_inclined_distance = distance_buffer[2];
        Left_inclined_distance = distance_buffer[4];
        Front_distance = (Front_distance_1 + Front_distance_2) / 2.0f;

        Right_encoder_count = Get_Right_Encoder();
        Left_encoder_count  = Get_Left_Encoder();

        if (Right_levelling == 1) {
            if (Right_inclined_distance > 20.0f) {
                if (Right_encoder_count > Right_inclined_overshoot_count) {
                    Right_levelling = 0;
                    levelling_changed = 1;
                }
            } else {
                Right_inclined_overshoot_count = nav.right_incl_overshoot + Right_encoder_count;
            }
        }

        if (Left_levelling == 1) {
            if (Left_inclined_distance > 20.0f) {
                if (Left_encoder_count > Left_inclined_overshoot_count) {
                    Left_levelling = 0;
                    levelling_changed = 1;
                }
            } else {
                Left_inclined_overshoot_count = nav.left_incl_overshoot + Left_encoder_count;
            }
        }

        if (Front_distance < nav.front_contact_distance) {
            if (Front_barrier_available == 0) {
                if (((Right_encoder_count + Left_encoder_count) / 2) > Front_bounce_count) {
                    Front_barrier_available = 1;
                    front_barrier_appearing = 1;
                    front_barrier_appearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.front_appearing_gap;
                }
            }
        } else {
            Front_bounce_count = nav.front_bounce + ((Right_encoder_count + Left_encoder_count) / 2);
        }

        if (Right_distance > 20.0f) {
            if (Right_barrier_available == 1) {
                if (Right_encoder_count > Right_overshoot_count) {
                    Right_barrier_available = 0;
                    barrier_changed = 1;
                    side_barrier_disappearing = 1;
                    side_barrier_disappearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_disappearing_gap;
                }
            } else {
                Right_bounce_count = nav.right_bounce + Right_encoder_count;
            }
        } else {
            if (Right_barrier_available == 1) {
                Right_overshoot_count = nav.right_overshoot + Right_encoder_count;
            } else {
                if (Right_encoder_count > Right_bounce_count) {
                    Right_barrier_available = 1;
                    barrier_changed = 1;
                    side_barrier_appearing = 1;
                    side_barrier_appearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_appearing_gap;
                    Right_levelling = 1;
                    levelling_changed = 1;
                }
            }
        }

        if (Left_distance > 20.0f) {
            if (Left_barrier_available == 1) {
                if (Left_encoder_count > Left_overshoot_count) {
                    Left_barrier_available = 0;
                    side_barrier_disappearing = 1;
                    side_barrier_disappearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_disappearing_gap;
                }
            } else {
                Left_bounce_count = nav.left_bounce + Left_encoder_count;
            }
        } else {
            if (Left_barrier_available == 1) {
                Left_overshoot_count = nav.left_overshoot + Left_encoder_count;
            } else {
                if (Left_encoder_count > Left_bounce_count) {
                    Left_barrier_available = 1;
                    side_barrier_appearing = 1;
                    side_barrier_appearing_count = (Right_encoder_count + Left_encoder_count) / 2 + nav.side_appearing_gap;
                    Left_levelling = 1;
                    levelling_changed = 1;
                }
            }
        }

        spot_from_middle = 0;
        if (levelling_changed == 1)
        {
            if ((Right_levelling == 1) && (Left_levelling == 1)) {
                mode = 1;
            } else if ((Right_levelling == 1) && (Left_levelling == 0)) {
                mode = 2;
                spot_from_right = Right_distance;
            } else if ((Right_levelling == 0) && (Left_levelling == 1)) {
                mode = 3;
                spot_from_left = Left_distance;
            } else {
                mode = 4;   // no walls: gyro-only straight
            }

            /* reset wall PID */
            integral = 0;
            derivative = 0;
            last_error = 0;

            /* reset gyro heading reference for the new mode */
            yaw = 0.0f;
            yaw_integral = 0.0f;

            levelling_changed = 0;
        }

        if (front_barrier_appearing == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > front_barrier_appearing_count) {
                to_take_decisions = 1;
            }
        } else if (side_barrier_disappearing == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > side_barrier_disappearing_count) {
                to_take_decisions = 1;
            }
        } else if (side_barrier_appearing == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > side_barrier_appearing_count) {
                to_take_decisions = 1;
            }
        } else if (barrier_unchanging == 1) {
            if (((Right_encoder_count + Left_encoder_count) / 2) > barrier_unchanged_count) {
                to_take_decisions = 1;
            }
        }

        if (to_take_decisions == 1) {
            if ((Right_barrier_available == 1) && (Left_barrier_available == 1) && (Front_barrier_available == 0)) {
                front_barrier_appearing = 0;
                side_barrier_appearing = 0;
                side_barrier_disappearing = 0;
                barrier_unchanging = 0;
                to_take_decisions = 0;
            } else {
                running = 0;
                continue;
            }
        }

        /* ---------------- wall PID ---------------- */
        float output;

        if (mode == 1) {
            float Kp = nav.mode1_kp;
            float Ki = nav.mode1_ki;
            float Kd = nav.mode1_kd;

            error = Left_distance - Right_distance - spot_from_middle;
            integral += error;
            derivative = error - last_error;
            output = Kp * error + Ki * integral + Kd * derivative;
            last_error = error;
        }
        else if (mode == 2) {
            float Kp = nav.mode2_kp;
            float Ki = nav.mode2_ki;
            float Kd = nav.mode2_kd;

            error = spot_from_right - Right_distance;

            integral += error;
            derivative = error - last_error;
            output = Kp * error + Ki * integral + Kd * derivative;
            last_error = error;
        }
        else if (mode == 3) {
            float Kp = nav.mode3_kp;
            float Ki = nav.mode3_ki;
            float Kd = nav.mode3_kd;

            error = Left_distance - spot_from_left;

            integral += error;
            derivative = error - last_error;
            output = Kp * error + Ki * integral + Kd * derivative;
            last_error = error;
        }
        else {
            output = 0;
        }

        /* ---------------- gyro ---------------- */
        if (MPU6050_Read_Gyro(&hi2c2, &MPU6050) != HAL_OK)
        {
            HAL_Delay(1);
            continue;
        }

        now = HAL_GetTick();
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        if (dt <= 0.0f)
            continue;

        float rate = GYRO_SIGN * ((float)MPU6050.Gz - gyro_z_offset);

        if (!rate_init) {
            rate_filt = rate;
            rate_init = 1;
        } else {
            rate_filt = (1.0f - RATE_FILT_ALPHA) * rate_filt + RATE_FILT_ALPHA * rate;
        }

        yaw += rate * dt * GYRO_SCALE;      // + = drifted left (CCW)

        yaw_integral += yaw * dt;
        if (yaw_integral >  50.0f) yaw_integral =  50.0f;
        if (yaw_integral < -50.0f) yaw_integral = -50.0f;

        float corr;
        if (mode == 4) {
            /* no walls: full gyro heading hold */
            corr = STRAIGHT_KP * yaw + STRAIGHT_KI * yaw_integral + STRAIGHT_KD * rate_filt;
        } else {
            /* walls steer the robot, gyro only damps wobble */
            corr = STRAIGHT_KD * rate_filt;
        }

        if (corr >  STRAIGHT_MAX_CORR) corr =  STRAIGHT_MAX_CORR;
        if (corr < -STRAIGHT_MAX_CORR) corr = -STRAIGHT_MAX_CORR;

        /* ---------------- motors ---------------- */

        float l, r;

        if ((fabsf(output) > 10.0f)||(Right_inclined_distance < 3.0)||(Left_inclined_distance < 3.0)) {
            l = (float)danger_speed - output;
            r = (float)danger_speed + output;
        } else {
            /* ---- speed profile: fast on straights, slow near the end ---- */
            float target_now = (float)base_speed;

        	if (fast_accel_enabled && straight_cells >= FAST_MIN_CELLS)
        	{
        	    float fast = (float)base_speed + FAST_EXTRA_SPEED;
        	    if (fast > FAST_MAX_SPEED) fast = FAST_MAX_SPEED;

        	    int32_t travelled = (Right_encoder_count + Left_encoder_count) / 2;
        	    int32_t remaining = (int32_t)(straight_cells * count_per_cell) - travelled;

        	    if (remaining > FAST_DECEL_TICKS)
        	        target_now = fast;
        	    else if (remaining > 0)
        	        target_now = base_speed + (fast - base_speed) * ((float)remaining / FAST_DECEL_TICKS);
        	}

            if (fast_cur_speed < target_now) {
                fast_cur_speed += FAST_ACCEL_STEP;
                if (fast_cur_speed > target_now) fast_cur_speed = target_now;
            } else if (fast_cur_speed > target_now) {
                fast_cur_speed -= FAST_DECEL_STEP;
                if (fast_cur_speed < target_now) fast_cur_speed = target_now;
            }

            if (fast_cur_speed >= 58.0f && fast_cur_speed <= 65.0f) {
                nav = speed_profiles[4];
            }else if(fast_cur_speed >= 66.0f && fast_cur_speed <= 70.0f){
            	nav = speed_profiles[5];
            }

            l  = (int16_t)fast_cur_speed - 0.5*(int16_t)output + corr;
            r = (int16_t)fast_cur_speed + 0.5*(int16_t)output - corr;
        }

        if (l > PWM_MAX) l = PWM_MAX;
        if (l < PWM_MIN) l = PWM_MIN;
        if (r > PWM_MAX) r = PWM_MAX;
        if (r < PWM_MIN) r = PWM_MIN;

        Left_Motor_Control((int16_t)l);
        Right_Motor_Control((int16_t)r);

        HAL_Delay(10);
    }

    Right_encoder_count = Get_Right_Encoder();
    Left_encoder_count  = Get_Left_Encoder();
    int16_t total_count = (Right_encoder_count + Left_encoder_count) / 2;
    float count = total_count / count_per_cell;
    int r = (int)(count + 0.5f);

    *NRBA = Right_barrier_available;
    *NLBA = Left_barrier_available;
    *NFBA = Front_barrier_available;
    *units = (uint8_t)r;
}

void updateFloodFillFastRun(void)
{
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            maze[x][y].distance = 255;

    maze[7][7].distance = 0;
    maze[7][8].distance = 0;
    maze[8][7].distance = 0;
    maze[8][8].distance = 0;

    uint8_t updated = 1;
    while (updated)
    {
        updated = 0;

        for (int x = 0; x < MAZE_SIZE; x++)
        {
            for (int y = 0; y < MAZE_SIZE; y++)
            {
                if (!maze[x][y].visited)
                    continue;

                uint8_t curDist = maze[x][y].distance;

                if (y < MAZE_SIZE - 1 &&
                    !maze[x][y].walls.up &&
                    maze[x][y + 1].visited &&
                    maze[x][y + 1].distance + 1 < curDist)
                {
                    maze[x][y].distance = maze[x][y + 1].distance + 1;
                    updated = 1;
                }

                if (x < MAZE_SIZE - 1 &&
                    !maze[x][y].walls.right &&
                    maze[x + 1][y].visited &&
                    maze[x + 1][y].distance + 1 < maze[x][y].distance)
                {
                    maze[x][y].distance = maze[x + 1][y].distance + 1;
                    updated = 1;
                }

                if (y > 0 &&
                    !maze[x][y].walls.down &&
                    maze[x][y - 1].visited &&
                    maze[x][y - 1].distance + 1 < maze[x][y].distance)
                {
                    maze[x][y].distance = maze[x][y - 1].distance + 1;
                    updated = 1;
                }

                if (x > 0 &&
                    !maze[x][y].walls.left &&
                    maze[x - 1][y].visited &&
                    maze[x - 1][y].distance + 1 < maze[x][y].distance)
                {
                    maze[x][y].distance = maze[x - 1][y].distance + 1;
                    updated = 1;
                }
            }
        }
    }
}

void TURN(int16_t target_ticks)
{
    int16_t right_enc, left_enc;
    int16_t turn_ticks;
    uint32_t time_to_release = 0;
    uint8_t first_reached = 1;

    // Bounds matching your PWM period context (999 maximum auto-reload)
    const int max_pwm = 60; //500
    const int min_pwm = 15; //80
    const int deadband = 15; // Tighter window for exact angle validation //5

    turn_pid.integral = 0.0f;
    turn_pid.prev_error = 0.0f;

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    while (1)
    {
        if (searching_mode == 2 && Button_UpdateMulti(2)) {
            stop_requested = 1;
            Left_Motor_Control(0);
            Right_Motor_Control(0);
            break;
        }

        left_enc  = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        right_enc = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);

        turn_ticks = (right_enc - left_enc) / 2;
        float error = (float)(target_ticks - turn_ticks);

        if ((error <= deadband) && (error >= -deadband))
        {
            if (first_reached == 1)
            {
                time_to_release = HAL_GetTick() + 200; // Let it stabilize for 200ms
                first_reached = 0;
            }
            else if (HAL_GetTick() > time_to_release)
            {
                break;
            }
        }
        else
        {
            first_reached = 1;
        }

        float output = PID_Compute(&turn_pid, error);
        int16_t pwm_magnitude = (int16_t)abs((int)output);

        if (pwm_magnitude > max_pwm) pwm_magnitude = max_pwm;
        if (pwm_magnitude < min_pwm) pwm_magnitude = min_pwm;

        if (output > 0)
        {
            Left_Motor_Control(-pwm_magnitude);
            Right_Motor_Control(pwm_magnitude);
        }
        else
        {
            Left_Motor_Control(pwm_magnitude);
            Right_Motor_Control(-pwm_magnitude);
        }

        HAL_Delay(10);
    }

    Left_Motor_Control(0);
    Right_Motor_Control(0);
}

void TURN_180(void)
{
    float right_sum = 0.0f;
    float left_sum = 0.0f;
    const int SAMPLES = 5;

 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_Delay(50);

    for (int sample = 0; sample < SAMPLES; sample++)
    {
        float distance_buffer[6];

        for (int i = 0; i < 6; i++) {
            float voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
            if (voltage > 0.24f)
                distance_buffer[i] = 4.9f / (voltage - 0.075f);
            else
                distance_buffer[i] = 30.0f;
        }

        float right_distance = distance_buffer[0];
        float left_distance  = distance_buffer[5];

        right_sum += right_distance;
        left_sum  += left_distance;

        HAL_Delay(2);
    }

    float Right_distance = right_sum / (float)SAMPLES;
    float Left_distance  = left_sum  / (float)SAMPLES;

    if (Right_distance > Left_distance)
    {
        TURN_GYRO(-180);
    }
    else
    {
        TURN_GYRO(180);
    }

	HAL_Delay(10);
	//MoveBackward_Encoder(200);
}

void Calibrate_At_Wall(void)
{
    const float TARGET   = 2.5f;
    const float Kd       = 2.0f;
    const int16_t base_speed = target_speed - 5;

    while (1) {
        float distance_buffer[6];
        for (int i = 0; i < 6; i++) {
            float voltage = (adc_buffer[i] * 3.3f) / 4095.0f;
            distance_buffer[i] = (voltage > 0.24f) ? 4.9f / (voltage - 0.075f) : 30.0f;
        }

        float RF_distance = distance_buffer[3];
        float LF_distance = distance_buffer[1];

        if (RF_distance <= TARGET && LF_distance <= TARGET)
            break;

        if (RF_distance > LF_distance) {
            Left_Motor_Control(0);
            int16_t spd = base_speed + (int16_t)((RF_distance - LF_distance) * Kd);
            if (spd > PWM_MAX) spd = PWM_MAX;
            if (spd < PWM_MIN) spd = PWM_MIN;
            Right_Motor_Control(spd);
        } else {
            Right_Motor_Control(0);
            int16_t spd = base_speed + (int16_t)((LF_distance - RF_distance) * Kd);
            if (spd > PWM_MAX) spd = PWM_MAX;
            if (spd < PWM_MIN) spd = PWM_MIN;
            Left_Motor_Control(spd);
        }

        HAL_Delay(5);
    }

    STOP(1);
}

void STOP(int16_t target_ticks){
	int16_t right, left;
	int16_t stop_ticks;

	uint32_t time_to_release;
	uint8_t first_reached = 1;

	float error, last_error = 0;
	float derivative;
	float output;

	const float Kp = 4.0f;
	const float Ki = 0.0f;
	const float Kd = 1.5f;

	const int max_pwm = 45;
	const int min_pwm = 15;
	const int deadband = 15;

	int16_t right_start_count = Get_Right_Encoder();
	int16_t left_start_count  = Get_Left_Encoder();

	while(1){
		right = Get_Right_Encoder() - right_start_count;
		left  = Get_Left_Encoder() - left_start_count;

		stop_ticks = (right + left) / 2;
		error = target_ticks - stop_ticks;

		if ((error <= deadband) && (error >= ((-1)*deadband))){
		     if(first_reached == 1){
		          time_to_release = HAL_GetTick() + 300;
		          first_reached = 0;
		     }
		     else{
		          uint32_t current_time = HAL_GetTick();
		          if(current_time > time_to_release){
		        			break;
		          }
		     }

		}
		derivative = error - last_error;
		output = Kp * error + Kd * derivative;
		last_error = error;

		int pwm;
		if (output > 0){
		     pwm = (int)output;
		}
		else{
		     pwm = (int)((-1)*output);
		}

		if (pwm > max_pwm) pwm = max_pwm;
		if (pwm < min_pwm) pwm = min_pwm;

		if (output > 0)
		{

	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

		}
		else
		{

	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		 }

		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);

    }
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void SPEED_SETTING(void)
{
	uint8_t selected_option = 0;
	Reset_Encoders();

	while(1){

	    uint32_t count = abs(Get_Right_Encoder());

	    if(count<10){
	        DISPLAY("SPEED","SETTING");
	    }else if((10<=count)&&(count<100)){
	        DISPLAY("35","STEADY");
	        selected_option = 0;
	    }else if((100<=count)&&(count<200)){
	        DISPLAY("40","BRISK");
	        selected_option = 1;
	    }else if((200<=count)&&(count<300)){
	        DISPLAY("45","FAST");
	        selected_option = 2;
	    }else if((300<=count)&&(count<400)){
	        DISPLAY("50","RACER");
	        selected_option = 3;
	    }else if((400<=count)&&(count<500)){
	        DISPLAY("60","TURBO");
	        selected_option = 4;
	    }else{
	        Reset_Encoders();
	    }

	    if (Button_UpdateMulti(0)){
	             break;
	    }
	}

	if(selected_option==0){
	    target_speed = 35;
	    nav = speed_profiles[0];
	}else if(selected_option==1){
	    target_speed = 40;
	    nav = speed_profiles[1];
	}else if(selected_option==2){
	    target_speed = 45;
	    nav = speed_profiles[2];
	}else if(selected_option==3){
	    target_speed = 50;
	    nav = speed_profiles[3];
	}else{
	    target_speed = 60;
	    nav = speed_profiles[4];
	}

	DISPLAY("SPEED","SETTED");
	HAL_Delay(500);
	DISPLAY("TEAM","NEURIX");
	return;
}

void Display_Counter(uint16_t delay){
	DISPLAY("READY", "...");
	HAL_Delay(delay);
	DISPLAY("SET", "...");
	HAL_Delay(delay);
	DISPLAY("GO!", "...");
	HAL_Delay(delay);
}

void LED_Loop_NonBlocking(void)
{
    uint32_t current_time = HAL_GetTick();

    // Define intervals
    uint32_t interval = (led_state < 6) ? 100 : 3000;

    // Check if the required time interval has passed
    if (current_time - last_led_time >= interval)
    {
        last_led_time = current_time; // Reset timer milestone

        if (led_state < 6)
        {
            // Even states (0, 2, 4) -> Turn ON. Odd states (1, 3, 5) -> Turn OFF.
            if (led_state % 2 == 0) {
                HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
            }
            led_state++;
        }
        else // State 6: The 3-second delay has finished
        {
            led_state = 0; // Restart the entire sequence
        }
    }
}
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // Check if this interrupt came from your dedicated 10ms control timer (e.g., TIM3)
    if (htim->Instance == TIM4) {

        speedProfile();
    }
}
*/

void I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    uint8_t i;
    HAL_StatusTypeDef result;

    printf("Scanning I2C bus...\r\n");

    for (i = 1; i < 128; i++)
    {
        /* HAL expects 7-bit address left-shifted by 1 */
        result = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i << 1), 2, 10);

        if (result == HAL_OK)
        {
            printf("I2C device found at address: 0x%02X\r\n", i);
        }
    }

    printf("I2C scan complete\r\n");
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
