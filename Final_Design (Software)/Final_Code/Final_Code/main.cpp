// Ei Jinish Valoi Perar!

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <math.h>
#include <time.h>

// Define pins for LED matrix rows and columns
#define ROW_PORT PORTD
#define ROW_DDR DDRD
#define COL_PORT PORTB
#define COL_DDR DDRB

// Define pins for buttons
#define LEFT_BUTTON_PIN PA0
#define RIGHT_BUTTON_PIN PA1
#define SCORE_BUTTON_PIN PA2
#define LIFE_BUTTON_PIN PA3
#define START_BUTTON_PIN PA4
#define RESET_BUTTON PA5

//Maximum Bullet at a time
#define MAX_BULLETS 5
#define LEFT_SHOOT_PIN PD0
#define RIGHT_SHOOT_PIN PD1

static volatile int pulse = 0;
static volatile int i = 0;

// Define SPI pins
#define MOSI PB5
#define SCK PB7

// Define LED matrix control pins
#define LOAD_PIN PB4 // Chip select pin for the MAX7219 modules

// MAX7219 register addresses
#define REG_NOOP 0x00
#define REG_DECODE_MODE 0x09
#define REG_INTENSITY 0x0A
#define REG_SCAN_LIMIT 0x0B
#define REG_SHUTDOWN 0x0C
#define REG_DISPLAY_TEST 0x0F

void reset() // to reset the whole code
{
  PORTA |= (1 << RESET_BUTTON);
  _delay_ms(10);
  wdt_enable(WDTO_15MS); // Enable watchdog timer with a timeout of 15ms
  while (1)
    ; // Wait for the watchdog timer to reset the microcontroller
}

// LED DRIVER FROM HERE
// Function to initialize SPI communication
void SPI_init()
{
  // Set MOSI and SCK as output
  DDRB |= (1 << MOSI) | (1 << SCK);
  // Enable SPI, set as master, and clock to fosc/16
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

// Function to send data via SPI
void SPI_send(uint8_t data)
{
  // Start transmission
  SPDR = data;
  // Wait for transmission complete
  while (!(SPSR & (1 << SPIF)))
    ;
}

// Function to send data to MAX7219
void MAX7219_send(uint8_t address, uint8_t data)
{
  // Select MAX7219 module
  PORTB &= ~(1 << LOAD_PIN);
  // Send address and data
  SPI_send(address);
  SPI_send(data);

  SPI_send(address);
  SPI_send(data);

  SPI_send(address);
  SPI_send(data);

  SPI_send(address);
  SPI_send(data);

  // DeSelect MAX7219 module
  PORTB |= (1 << LOAD_PIN);
}
void MAX7219_send2(uint8_t address, uint8_t data)
{
  // Send address and data
  SPI_send(address);
  SPI_send(data);
}

void select_Load()
{
  // Select MAX7219 module
  PORTB &= ~(1 << LOAD_PIN);
}
void deSelect_Load()
{
  // DeSelect MAX7219 module
  PORTB |= (1 << LOAD_PIN);
}
// Function to initialize MAX7219 driver
void initMAX7219()
{
  // Set decode mode: no decode for all digits
  MAX7219_send(REG_DECODE_MODE, 0x00);

  // Set intensity (brightness) level
  MAX7219_send(REG_INTENSITY, 0x0F); // Max intensity

  // Set scan limit: use all digits (rows)
  MAX7219_send(REG_SCAN_LIMIT, 0x07); // For an 8x8 matrix

  // Enable display
  MAX7219_send(REG_SHUTDOWN, 0x01); // Normal operation mode
}
// LED DRIVER ENDS HERE

// Function to initialize GPIO pins
void init_pins()
{
  // Set LED matrix row pins as output
  ROW_DDR = 0xFF;
  // Set LED matrix column pins as output
  COL_DDR = 0xFF;
  // Enable internal pull-up resistors for button pins
  DDRA = 0b11110100;
  DDRD = 0b11111100;
}

int life = 3;
int level;
int brick_count;
#define dimension 16

// bool bricks[dimension][dimension] = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//                                     {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
//                                     {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
//                                     {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
//                                     {1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1},
//                                     {0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0},
//                                     {0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0},
//                                     {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}};
bool bricks[dimension][dimension];
// put brick in the display anywhere

struct Wall
{
  int left = 0;
  int right = 15;
  int up = 0;
  int down = 15;

  int center_col() { return (left + right) / 2; } // returns the center column, useful for paddle initial position
} wall;

struct Paddle
{
  int position; // position of the center of the paddle
  int size;
  int row;

  void init() { position = wall.center_col(); }
  Paddle()
  {
    size = 5;
    row = wall.down;
    init();
  }

  int left() { return position - size / 2; }
  int right() { return position + size / 2; }

  void goleft()
  {
    if (left() > wall.left)
      position--;
  }
  void goright()
  {
    if (right() < wall.right)
      position++;
  }
} paddle;

struct Ball
{
  int row, col;
  int row_dir, col_dir;

  void init() // sets the starting position of the ball
  {
    row = wall.down;
    col = paddle.position;
    row_dir = -1;
    col_dir = 0;
  }

  Ball() { init(); }

  int nextcol() { return col + col_dir; }
  int nextrow() { return row + row_dir; }
  void update() { row = nextrow(), col = nextcol(); }

  void collision_check()
  {
    // Check for collision with walls
    if (row <= wall.up)
      row_dir = -row_dir; // Reflect ball downwards
    if (col <= wall.left || col >= wall.right)
      col_dir = -col_dir; // Reflect ball horizontally

    // Check for collision with paddle
    if (nextrow() == paddle.row && col >= paddle.left() && col <= paddle.right())
    {
      if (col < paddle.position)
      {
        row_dir = -1;
        col_dir = -1;
      }
      else if (col == paddle.position)
      {
        row_dir = -1;
        col_dir = 0;
      }
      else if (col > paddle.position)
      {
        row_dir = -1;
        col_dir = 1;
      }
    }

    // collision with bricks
    int nrow = nextrow();
    if (nrow < dimension && bricks[nrow][nextcol()])
    {
      PORTA |= (1 << SCORE_BUTTON_PIN); // to sound the beeper
      bricks[nextrow()][nextcol()] = 0;
      --brick_count;
      row_dir = -row_dir;
      _delay_ms(5);
    }
  }
} ball;



//---------------------------------//
//code for Shooting the bricks
int bullets_counter = 0;  //for controlling the speed of the bullet

// Define bullet structure
typedef struct {
    int row;
    int col;
    int active;
} Bullet;

// Array to store bullets
Bullet bullets[MAX_BULLETS];

bool prev_left_shoot = false, prev_right_shoot = false;
bool left_shoot, right_shoot;

// Update bullet positions

void update_bullets(){

	// Read button inputs
	left_shoot = PIND & (1 << LEFT_SHOOT_PIN);
	right_shoot = PIND & (1 << RIGHT_SHOOT_PIN);

    // Fire bullets when buttons are pressed
    if (!(prev_left_shoot) && left_shoot) {
         // Find an inactive bullet slot
        for (int i = 0; i < MAX_BULLETS; i++) {
            if (!bullets[i].active) {
                // Activate the bullet
                bullets[i].row = 15; 
                bullets[i].col = paddle.left(); 
                bullets[i].active = 1;
                break;
            }
        }
    }

    if (!(prev_right_shoot) && right_shoot) {
        for (int i = 0; i < MAX_BULLETS; i++) {
            if (!bullets[i].active) {
                // Activate the bullet
                bullets[i].row = 15;
                bullets[i].col = paddle.right();
                bullets[i].active = 1;
                break;
            }
        }
    }
	prev_left_shoot = left_shoot;
	prev_right_shoot = right_shoot;

	// Update bullet positions
	for (int i = 0; i < MAX_BULLETS; i++) {
      if (bullets[i].active) {

          // Move the bullet upwards
          bullets[i].row--;

          // Check if bullet reached the top or hit a brick
          if (bullets[i].row < 0) {
              bullets[i].active = 0;
				      continue;
          } 
			    else if (bricks[bullets[i].row][bullets[i].col]) {
				      bricks[bullets[i].row][bullets[i].col] = 0;
              --brick_count;
				      bullets[i].active = 0;
			    }
      }
  }
}
//---------------------------------//






struct Display
{
  int rows = dimension;
  int cols = dimension;
  int splitrows = dimension / 2;
  int splitcols = dimension / 2;

  bool ara[17][17];
  int split1[10];
  int split2[10];
  int split3[10];
  int split4[10]; // 10 for out_of_bound safety

  void resetArray()
  {
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        ara[i][j] = 0;
  }
  void setArray(bool (*ara1)[dimension])
  {
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        ara[i][j] = ara1[i][j];
  }
  Display() { resetArray(); } // initializing with 0

  void bricks_in_display()
  {
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        ara[i][j] = bricks[i][j];
  }
  void paddle_in_display()
  {
    for (int i = paddle.left(); i <= paddle.right(); ++i)
      ara[paddle.row][i] = 1;
  }
  void ball_in_display()
  {
    if (ball.row <= wall.down)
      ara[ball.row][ball.col] = 1;
  }

  void bullet_in_display() {
		for (int i = 0; i < MAX_BULLETS; i++) {
			if (bullets[i].active && bullets[i].row>=0) {
				ara[bullets[i].row][bullets[i].col] = 1;
			}
		}
	}

  void split()
  {
    // putting first 8*8 portion of display into split1
    for (int i = 0; i < splitrows; i++)
    {
      split1[i] = 0;
      for (int j = 0; j < splitcols; j++)
        split1[i] |= (ara[i][j] << (splitcols - 1 - j)); // Using bitmasking to transform columns into a single integer
    }
    // second portion to split2
    for (int i = 0; i < splitrows; i++)
    {
      split2[i] = 0;
      for (int j = cols - splitcols; j < cols; j++)
        split2[i] |= (ara[i][j] << (cols - j - 1));
    }
    // split3
    for (int i = rows - splitrows; i < rows; i++)
    {
      split3[i - rows + splitrows] = 0;
      for (int j = 0; j < splitcols; j++)
        split3[i - rows + splitrows] |= (ara[i][j] << (splitcols - 1 - j));
    }
    // last portion to split4
    for (int i = rows - splitrows; i < rows; i++)
    {
      split4[i - rows + splitrows] = 0;
      for (int j = cols - splitcols; j < cols; j++)
        split4[i - rows + splitrows] |= (ara[i][j] << (cols - j - 1));
    }
  }

  void turnoff()
  {
    // Turn off all LEDs
    for (uint8_t i = 1; i <= 8; i++)
      MAX7219_send(i, 0x00);
  }

  void send_to_driver(bool staticState = false)
  {
    split();
    for (uint8_t i = 1; i <= 8; i++)
    {
      select_Load();

      MAX7219_send2(i, split4[i - 1]);
      MAX7219_send2(i, split3[i - 1]);
      MAX7219_send2(i, split2[i - 1]);
      MAX7219_send2(i, split1[i - 1]);

      deSelect_Load();
    }

    // Turn off all LEDs
    if (!staticState)
      turnoff();
    // if staticState is true, display will not reset
  }
} display;

void show_message(bool (*ara)[dimension], bool wait_for_button = true)
{
  display.setArray(ara);
  display.send_to_driver(true);
  if (wait_for_button)
    while (!(PINA & (1 << START_BUTTON_PIN)))
      ; // display untill start button pressed
  else
    _delay_ms(1000);
  display.turnoff();
  _delay_ms(500);
  display.resetArray();
}

int speed = 8;
int bullet_speed = 3;
void level_up()
{
  level++;
  if(speed >= 0)
    speed -= 2;
  brick_count = 0;
  if (level == 1)
  {
    // parallel line pattern
    for (int i = 0; i < dimension / 2; i++)
      for (int j = 0; j < dimension; j++)
      {
        if ((i + j) % 2 == 0 || (i + j) % 3 == 0)
          bricks[i][j] = 0;
        else
          bricks[i][j] = 1, brick_count++;
      }
  }
  else if (level == 2)
  {
    // pyramid pattern
    for (int i = 0; i < dimension / 2; i++)
      for (int j = 0; j < dimension; j++)
      {
        if ((i >= j && i < (dimension - j)) || (i < j && i >= (dimension - j - 1)))
          bricks[i][j] = 0;
        else
          bricks[i][j] = 1, brick_count++;
      }
  }
  else if (level == 3)
  {
    // checkboard pattern
    for (int i = 0; i < dimension / 2; i++)
      for (int j = 0; j < dimension; j++)
      {
        if ((i + j) % 2 == 0)
          bricks[i][j] = 1, brick_count++;
        else
          bricks[i][j] = 0;
      }
  }
  else
  {
    // random pattern
    // Adjust the probability of brick generation based on the level
    // Start with 50% probability and increase by 10% for each level above 3
    double brick_probability = 0.5 + (level - 4) * 0.1;

    for (int i = 0; i < dimension / 2 + level - 3; i++)
      for (int j = 0; j < dimension; j++)
      {
        if ((rand() / (double)RAND_MAX) <= brick_probability)
        {
          bricks[i][j] = 1;
          brick_count++;
        }
        else
          bricks[i][j] = 0;
      }
  }

  paddle.init();
  ball.init();
  _delay_ms(500);
}

void game_init()
{
  level_up();
  
  // Initialize bullets
  for (int i = 0; i < MAX_BULLETS; i++) {
    bullets[i].active = 0; // Bullet is inactive initially
  }

  bool get_set_go[dimension][dimension] = {
      {0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0},
      {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
      {1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0},
      {1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0},
      {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
      {1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
      {1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0},
      {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0},
      {0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0}};
  show_message(get_set_go);
  // commenting out 3, 2, 1 ... because ram is not enough :(
  // bool ara3[dimension][dimension] = {
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  //   };
  // show_message(ara3, false);
  // bool ara2[dimension][dimension] = {
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  //   };
  // show_message(ara2, false);
  // bool ara1[dimension][dimension] = {
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
  //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  //   };
  // show_message(ara1, false);
}
void game_over()
{
  bool game_over_ara[dimension][dimension] = {
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1},
      {1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0},
      {1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0},
      {1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0},
      {0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1},
      {1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1},
      {1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1},
      {1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0},
      {0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  show_message(game_over_ara);
  reset(); // start all over
}

int main()
{
  init_pins();

  // LED DRIVER FROM HERE
  // Initialize SPI communication
  SPI_init();

  // Set LOAD_PIN as output
  DDRB |= (1 << LOAD_PIN);
  // Set LOAD_PIN high (deselect MAX7219 modules)
  PORTB |= (1 << LOAD_PIN);

  // Initialize MAX7219 drivers
  initMAX7219();
  // LED DRIVER ENDS HERE

  // for working on raising edge of push button
  bool prev_left_button = false, prev_right_button = false;
  bool left_button, right_button;
  bool prev_pause_button = false;
  bool pause_button;
  bool paused;

  int ball_counter = 0; // for slowing down the ball movement

  game_init();

  // Main game loop
  while (1)
  {
    // Read button inputs
    left_button = PINA & (1 << LEFT_BUTTON_PIN);
    right_button = PINA & (1 << RIGHT_BUTTON_PIN);
    pause_button = PINA & (1 << START_BUTTON_PIN);

    // pause and resume
    if (!prev_pause_button && pause_button)
      paused = !paused;
    prev_pause_button = pause_button;
    if (paused)
    {
      display.send_to_driver();
      continue;
    }

    // paddle movement
    if (!prev_left_button && left_button) // raising edge
      paddle.goleft();
    if (!prev_right_button && right_button)
      paddle.goright();
    prev_left_button = left_button;
    prev_right_button = right_button;

    // Update ball position
    ball_counter++;
    if (ball_counter >= speed)
    {
      ball.update();
      ball.collision_check();

      // check if bricks finished
      if (!brick_count)
        level_up();

      // check if ball gone down
      if (ball.row >= wall.down + 5)
      {
        if (life)
        {
          life--;
          PORTA |= (1 << LIFE_BUTTON_PIN); // to sound the beeper
          ball.init();                     // reviving if life remains
        }
        else
          game_over();
      }

      ball_counter = 0;
    }

    bullets_counter++;
		if(bullets_counter >= bullet_speed){
			update_bullets();
			bullets_counter = 0;
		}

    // Update display buffer
    display.bricks_in_display();
    display.paddle_in_display();
    display.ball_in_display();
    display.bullet_in_display();

    // sending to led driver
    display.send_to_driver();

    // turning off beeper
    PORTA &= ~(1 << SCORE_BUTTON_PIN);
    PORTA &= ~(1 << LIFE_BUTTON_PIN);
    PORTA &= ~(1 << RESET_BUTTON);

    // Adjusted delay for slower game execution
    _delay_ms(5);
  }

  return 0;
}
