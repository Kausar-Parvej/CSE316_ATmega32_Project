// Ei Jinish Valoi Perar!

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <time.h>   // for rand
#include <string.h> // for memset

// Define pins for LED matrix rows and columns
#define ROW_PORT PORTD
#define ROW_DDR DDRD
#define COL_PORT PORTB
#define COL_DDR DDRB

// Define pins for buttons
#define LEFT_BUTTON_PIN PA0
#define RIGHT_BUTTON_PIN PA1
#define START_BUTTON_PIN PA4

#define SCORE_BUTTON_PIN PA2
#define LIFE_BUTTON_PIN PA3
#define RESET_BUTTON PA5

#define LIFE_GAIN_PIN PC7

// Maximum Bullet at a time
#define MAX_BULLETS 4
#define LEFT_SHOOT_PIN PA7
#define RIGHT_SHOOT_PIN PA6

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
  _delay_ms(200);
  // PORTA &= ~(1 << RESET_BUTTON);
  wdt_enable(WDTO_15MS); // Enable watchdog timer with a timeout of 15ms
  while (1)
    ; // Wait for the watchdog timer to reset the microcontroller
}

inline void disableJTAG()
{
  MCUCSR |= (1 << JTD); // First write
  MCUCSR |= (1 << JTD); // Second write within four clock cycles
}

// LED DRIVER FROM HERE
// Function to initialize SPI communication
inline void SPI_init()
{
  // Set MOSI and SCK as output
  DDRB |= (1 << MOSI) | (1 << SCK);
  // Enable SPI, set as master, and clock to fosc/16
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

// Function to send data via SPI
inline void SPI_send(uint8_t data)
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
inline void MAX7219_send2(uint8_t address, uint8_t data)
{
  // Send address and data
  SPI_send(address);
  SPI_send(data);
}

void select_Load() { PORTB &= ~(1 << LOAD_PIN); }  // Select MAX7219 module
void deSelect_Load() { PORTB |= (1 << LOAD_PIN); } // DeSelect MAX7219 module
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
  DDRA = 0;
  DDRA &= ~((1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN) | (1 << START_BUTTON_PIN));
  DDRA &= ~((1 << LEFT_SHOOT_PIN) | (1 << RIGHT_SHOOT_PIN));
  DDRA |= (1 << SCORE_BUTTON_PIN) | (1 << LIFE_BUTTON_PIN) | (1 << RESET_BUTTON);
  PORTA = 0;

  DDRC = 0;
  DDRC |= (1 << LIFE_GAIN_PIN);
}

int life = 3;
int level;
int brick_count;
#define dimension 16

// wrapper of 2d boolean to save space
struct bitarray
{
  uint16_t rows[17] = {0};

  struct Proxy
  {
    int row;
    Proxy(int x) { row = x; }
    bool operator[](size_t colIndex) { return 1 & (row >> colIndex); }
  };

  Proxy operator[](size_t rowIndex) { return Proxy(rows[rowIndex]); }

  // Function to set a specific bit
  void setBit(size_t rowIndex, size_t colIndex, bool value)
  {
    if (value)
      rows[rowIndex] |= (1 << colIndex);
    else
      rows[rowIndex] &= ~(1 << colIndex);
  }
};

bitarray bricks;
// put brick in the display anywhere..

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

  inline void goleft()
  {
    if (left() > wall.left)
      position--;
  }
  inline void goright()
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
  bool valid_index(int row, int col) { return row < dimension && col < dimension && row >= 0 && col >= 0; }
  void update() { row = nextrow(), col = nextcol(); }

  void collision_check()
  {
  collision:
    // Check for collision with paddle
    int ncol = nextcol();
    if (nextrow() == paddle.row &&
        ((ncol >= paddle.left() && ncol <= paddle.right()) ||
         (col >= paddle.left() && col <= paddle.right())))
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

      goto collision;
    }

    // Check for collision with walls
    if (nextrow() < wall.up)
    {
      row_dir = 1;
      goto collision; // Reflect ball downwards
    }
    if (nextcol() < wall.left)
    { // Reflect ball horizontally
      col_dir = 1;
      goto collision;
    }
    if (nextcol() > wall.right)
    {
      col_dir = -1;
      goto collision;
    }

    // collision with bricks
    int nrow = nextrow(), ncol2 = nextcol();
    if (valid_index(nrow, ncol2) && bricks[nrow][ncol2])
    {
      PORTA |= (1 << SCORE_BUTTON_PIN); // to sound the beeper
      _delay_ms(100);

      bricks.setBit(nrow, ncol2, 0);
      --brick_count;
      row_dir = -row_dir;
      goto collision;
    }
  }
} ball;

//---------------------------------//
// code for Shooting the bricks
int bullets_counter = 0; // for controlling the speed of the bullet

// Define bullet structure
typedef struct
{
  int row;
  int col;
  bool active;
} Bullet;

// Array to store bullets
Bullet bullets[MAX_BULLETS];

bool prev_left_shoot = false, prev_right_shoot = false;
bool left_shoot, right_shoot;

// Update bullet positions

void update_bullets()
{
  // Read button inputs
  left_shoot = PINA & (1 << LEFT_SHOOT_PIN);
  right_shoot = PINA & (1 << RIGHT_SHOOT_PIN);

  // Fire bullets when buttons are pressed
  if (!(prev_left_shoot) && left_shoot)
  {
    // Find an inactive bullet slot
    for (int i = 0; i < MAX_BULLETS; i++)
    {
      if (!bullets[i].active)
      {
        // Activate the bullet
        bullets[i].row = wall.down;
        bullets[i].col = paddle.left();
        bullets[i].active = true;
        break;
      }
    }
  }

  if (!(prev_right_shoot) && right_shoot)
  {
    for (int i = 0; i < MAX_BULLETS; i++)
    {
      if (!bullets[i].active)
      {
        // Activate the bullet
        bullets[i].row = wall.down;
        bullets[i].col = paddle.right();
        bullets[i].active = true;
        break;
      }
    }
  }
  prev_left_shoot = left_shoot;
  prev_right_shoot = right_shoot;

  // Update bullet positions
  for (int i = 0; i < MAX_BULLETS; i++)
  {
    if (bullets[i].active)
    {
      // Move the bullet upwards
      bullets[i].row--;

      // Check if bullet reached the top or hit a brick
      if (bullets[i].row < 0)
      {
        bullets[i].active = 0;
        continue;
      }
      else if (bricks[bullets[i].row][bullets[i].col])
      {
        PORTA |= (1 << SCORE_BUTTON_PIN); // to sound the beeper
        _delay_ms(100);
        bricks.setBit(bullets[i].row, bullets[i].col, 0);
        // PORTA &= ~(1 << SCORE_BUTTON_PIN);
        --brick_count;
        bullets[i].active = 0;
      }
    }
  }
}

// code for gaining lives
typedef struct
{
  int row;
  int col;
  int active;
} LifeGain;

LifeGain life_gain = {0, 0, 0}; // Initialize life_gain as inactive

// handle catching lives
void catch_life()
{
  if (life_gain.active && life_gain.row == paddle.row && (life_gain.col >= paddle.left() && life_gain.col <= paddle.right()))
  {
    life = life + 1;
    // code to sound the beeper
    PORTC |= (1 << LIFE_GAIN_PIN);
    _delay_ms(100);
    life_gain.active = 0;
  }
}

int life_gain_counter = 0; // for controlling the frequency of life gain
void init_life_gain()
{
  life_gain_counter++;
  if (life_gain.active || life >= 3)
  {
    life_gain_counter = 0; // to make the frequency of life gain constant
  }
  if (life_gain_counter == 300)
  {
    life_gain_counter = 0;

    if (life_gain.active == 0 && life < 3)
    {
      life_gain.active = 1;
      life_gain.row = 0;
      life_gain.col = 1 + (rand() % (dimension - 2)); // to esnsure the life gain pattern is within the wall
    }
  }
}

int life_gain_speed = 0;
void update_life_gain()
{
  life_gain_speed++;
  if (life_gain_speed >= 15)
  {
    if (life_gain.active)
    {
      life_gain.row++;
      if (life_gain.row >= wall.down + 5)
      {
        life_gain.active = 0;
      }
    }
    life_gain_speed = 0;
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

  void resetArray() { memset(ara, 0, sizeof ara); }
  inline void setArray(bool (*ara1)[dimension])
  {
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        ara[i][j] = ara1[i][j];
  }
  Display() { resetArray(); } // initializing with 0

  inline void bricks_in_display()
  {
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        ara[i][j] = bricks[i][j];
  }
  inline void paddle_in_display()
  {
    for (int i = paddle.left(); i <= paddle.right(); ++i)
      ara[paddle.row][i] = 1;
  }
  inline void ball_in_display()
  {
    if (ball.row <= wall.down)
      ara[ball.row][ball.col] = 1;
  }
  inline void bullet_in_display()
  {
    for (int i = 0; i < MAX_BULLETS; i++)
    {
      if (bullets[i].active && bullets[i].row >= 0)
        ara[bullets[i].row][bullets[i].col] = 1;
    }
  }
  void life_gain_in_display()
  {
    if (life_gain.active)
    {
      if (life_gain.row >= 0 && life_gain.row <= wall.down)
      {
        ara[life_gain.row][life_gain.col] = 1;
      }
      if (life_gain.row - 1 >= 0 && life_gain.row - 1 <= wall.down)
      {
        ara[life_gain.row - 1][life_gain.col] = 1;
        ara[life_gain.row - 1][life_gain.col - 1] = 1;
        ara[life_gain.row - 1][life_gain.col + 1] = 1;
      }
      if (life_gain.row - 2 >= 0 && life_gain.row - 2 <= wall.down)
      {
        ara[life_gain.row - 2][life_gain.col] = 1;
      }
    }
  }

  void split()
  {
    // putting first 8*8 portion of display into split3
    for (int i = 0; i < splitrows; i++)
    {
      split3[splitrows - i - 1] = 0;
      for (int j = 0; j < splitcols; j++)
        split3[splitrows - i - 1] |= (ara[i][j] << j); // Using bitmasking to transform columns into a single integer
    }
    // second portion to split2
    for (int i = 0; i < splitrows; i++)
    {
      split2[splitrows - i - 1] = 0;
      for (int j = 0; j < splitcols; j++)
        split2[splitrows - i - 1] |= (ara[i][cols - splitcols + j] << j);
    }
    // split4
    for (int i = 0; i < splitrows; i++)
    {
      split4[splitrows - i - 1] = 0;
      for (int j = 0; j < splitcols; j++)
        split4[splitrows - i - 1] |= (ara[rows - 1 - i][splitcols - 1 - j] << j);
    }
    // last portion to the first matrix
    for (int i = rows - splitrows; i < rows; i++)
    {
      split1[i - rows + splitrows] = 0;
      for (int j = cols - splitcols; j < cols; j++)
        split1[i - rows + splitrows] |= (ara[i][j] << (cols - j - 1));
    }
  }

  inline void turnoff()
  {
    // Turn off all LEDs
    for (uint8_t i = 1; i <= 8; i++)
      MAX7219_send(i, 0x00);
  }

  void send_to_driver()
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
  }
} display;

void show_message(bool (*ara)[dimension], bool wait_for_button = true)
{
  display.setArray(ara);
  display.send_to_driver();
  if (wait_for_button)
    while (!(PINA & (1 << START_BUTTON_PIN)))
      ; // display untill start button pressed
  else
    _delay_ms(500);
  display.turnoff();
  _delay_ms(300);
  display.resetArray();
}

int speed = 8;
int bullet_speed = 3;
void level_up()
{
  for (int i = 0; i < MAX_BULLETS; i++)
    bullets[i].active = false;

  level++;
  if (speed >= 0)
    speed -= 2;
  brick_count = 0;

  if (level == 2)
  {
    // parallel line pattern
    for (int i = 0; i < dimension / 2; i++)
      for (int j = 0; j < dimension; j++)
      {
        if ((i + j) % 2 == 0 || (i + j) % 3 == 0)
          bricks.setBit(i, j, 0);
        else
          bricks.setBit(i, j, 1), brick_count++;
      }
  }
  else if (level == 1)
  {
    // pyramid pattern
    for (int i = 0; i < dimension / 2; i++)
      for (int j = 0; j < dimension; j++)
      {
        if ((i >= j && i < (dimension - j)) || (i < j && i >= (dimension - j - 1)))
          bricks.setBit(i, j, 0);
        else
          bricks.setBit(i, j, 1), brick_count++;
      }
  }
  else if (level == 3)
  {
    // checkboard pattern
    for (int i = 0; i < dimension / 2; i++)
      for (int j = 0; j < dimension; j++)
      {
        if ((i + j) % 2 == 0)
          bricks.setBit(i, j, 1), brick_count++;
        else
          bricks.setBit(i, j, 0);
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
          bricks.setBit(i, j, 1), brick_count++;
        else
          bricks.setBit(i, j, 0);
      }
  }

  paddle.init();
  ball.init();
  _delay_ms(500);
}

bool get_set_go[dimension][dimension];

void game_init()
{
  PORTA |= (1 << RESET_BUTTON);
  level_up();

  // Initialize bullets
  for (int i = 0; i < MAX_BULLETS; i++)
    bullets[i].active = 0; // Bullet is inactive initially

  // showing get set go
  memset(get_set_go, 0, sizeof get_set_go);
  get_set_go[0][1] = 1, get_set_go[0][2] = 1, get_set_go[0][3] = 1;
  get_set_go[0][5] = 1, get_set_go[0][6] = 1, get_set_go[0][7] = 1;
  get_set_go[0][8] = 1, get_set_go[0][10] = 1, get_set_go[0][11] = 1;
  get_set_go[0][12] = 1, get_set_go[0][13] = 1, get_set_go[0][14] = 1;
  get_set_go[1][0] = 1, get_set_go[1][5] = 1, get_set_go[1][12] = 1;
  get_set_go[2][0] = 1, get_set_go[2][2] = 1, get_set_go[2][3] = 1;
  get_set_go[2][5] = 1, get_set_go[2][6] = 1, get_set_go[2][7] = 1;
  get_set_go[2][12] = 1, get_set_go[3][0] = 1, get_set_go[3][3] = 1;
  get_set_go[3][5] = 1, get_set_go[3][12] = 1, get_set_go[4][1] = 1;
  get_set_go[4][2] = 1, get_set_go[4][3] = 1, get_set_go[4][5] = 1;
  get_set_go[4][6] = 1, get_set_go[4][7] = 1, get_set_go[4][8] = 1;
  get_set_go[4][12] = 1, get_set_go[6][0] = 1, get_set_go[6][1] = 1;
  get_set_go[6][2] = 1, get_set_go[6][3] = 1, get_set_go[6][5] = 1;
  get_set_go[6][6] = 1, get_set_go[6][7] = 1, get_set_go[6][8] = 1;
  get_set_go[6][10] = 1, get_set_go[6][11] = 1, get_set_go[6][12] = 1;
  get_set_go[6][13] = 1, get_set_go[6][14] = 1, get_set_go[7][0] = 1;
  get_set_go[7][5] = 1, get_set_go[7][12] = 1, get_set_go[8][0] = 1;
  get_set_go[8][1] = 1, get_set_go[8][2] = 1, get_set_go[8][3] = 1;
  get_set_go[8][5] = 1, get_set_go[8][6] = 1, get_set_go[8][7] = 1;
  get_set_go[8][12] = 1, get_set_go[9][3] = 1, get_set_go[9][5] = 1;
  get_set_go[9][12] = 1, get_set_go[10][0] = 1, get_set_go[10][1] = 1;
  get_set_go[10][2] = 1, get_set_go[10][3] = 1, get_set_go[10][5] = 1;
  get_set_go[10][6] = 1, get_set_go[10][7] = 1, get_set_go[10][8] = 1;
  get_set_go[10][12] = 1, get_set_go[12][4] = 1, get_set_go[12][5] = 1;
  get_set_go[12][6] = 1, get_set_go[12][9] = 1, get_set_go[12][10] = 1;
  get_set_go[12][11] = 1, get_set_go[13][3] = 1, get_set_go[13][8] = 1;
  get_set_go[13][12] = 1, get_set_go[14][3] = 1, get_set_go[14][5] = 1;
  get_set_go[14][6] = 1, get_set_go[14][8] = 1, get_set_go[14][12] = 1;
  get_set_go[15][4] = 1, get_set_go[15][5] = 1, get_set_go[15][6] = 1;
  get_set_go[15][9] = 1, get_set_go[15][10] = 1, get_set_go[15][11] = 1;
  show_message(get_set_go);

  // showing 3
  memset(get_set_go, 0, sizeof get_set_go);
  get_set_go[1][5] = 1, get_set_go[1][6] = 1, get_set_go[1][7] = 1;
  get_set_go[2][4] = 1, get_set_go[2][5] = 1, get_set_go[2][6] = 1;
  get_set_go[2][7] = 1, get_set_go[2][8] = 1, get_set_go[3][4] = 1;
  get_set_go[3][8] = 1, get_set_go[3][9] = 1, get_set_go[4][9] = 1;
  get_set_go[4][10] = 1, get_set_go[5][9] = 1, get_set_go[5][10] = 1;
  get_set_go[6][8] = 1, get_set_go[6][9] = 1, get_set_go[7][6] = 1;
  get_set_go[7][7] = 1, get_set_go[7][8] = 1, get_set_go[8][8] = 1;
  get_set_go[8][9] = 1, get_set_go[9][9] = 1, get_set_go[9][10] = 1;
  get_set_go[10][9] = 1, get_set_go[10][10] = 1, get_set_go[11][4] = 1;
  get_set_go[11][8] = 1, get_set_go[11][9] = 1, get_set_go[12][4] = 1;
  get_set_go[12][5] = 1, get_set_go[12][6] = 1, get_set_go[12][7] = 1;
  get_set_go[12][8] = 1, get_set_go[13][5] = 1, get_set_go[13][6] = 1;
  get_set_go[13][7] = 1;
  show_message(get_set_go, false);

  // showing 2
  memset(get_set_go, 0, sizeof get_set_go);
  get_set_go[1][6] = 1, get_set_go[1][7] = 1, get_set_go[2][5] = 1;
  get_set_go[2][6] = 1, get_set_go[2][7] = 1, get_set_go[2][8] = 1;
  get_set_go[3][4] = 1, get_set_go[3][5] = 1, get_set_go[3][8] = 1;
  get_set_go[3][9] = 1, get_set_go[4][8] = 1, get_set_go[4][9] = 1;
  get_set_go[5][8] = 1, get_set_go[5][9] = 1, get_set_go[6][7] = 1;
  get_set_go[6][8] = 1, get_set_go[7][6] = 1, get_set_go[7][7] = 1;
  get_set_go[8][5] = 1, get_set_go[8][6] = 1, get_set_go[9][4] = 1;
  get_set_go[9][5] = 1, get_set_go[10][4] = 1, get_set_go[10][5] = 1;
  get_set_go[10][6] = 1, get_set_go[10][7] = 1, get_set_go[10][8] = 1;
  get_set_go[10][9] = 1, get_set_go[11][4] = 1, get_set_go[11][5] = 1;
  get_set_go[11][6] = 1, get_set_go[11][7] = 1, get_set_go[11][8] = 1;
  get_set_go[11][9] = 1;
  show_message(get_set_go, false);

  // showing 1
  memset(get_set_go, 0, sizeof get_set_go);
  get_set_go[1][7] = 1, get_set_go[1][8] = 1, get_set_go[2][6] = 1;
  get_set_go[2][7] = 1, get_set_go[2][8] = 1, get_set_go[3][5] = 1;
  get_set_go[3][6] = 1, get_set_go[3][7] = 1, get_set_go[3][8] = 1;
  get_set_go[4][7] = 1, get_set_go[4][8] = 1, get_set_go[5][7] = 1;
  get_set_go[5][8] = 1, get_set_go[6][7] = 1, get_set_go[6][8] = 1;
  get_set_go[7][7] = 1, get_set_go[7][8] = 1, get_set_go[8][7] = 1;
  get_set_go[8][8] = 1, get_set_go[9][7] = 1, get_set_go[9][8] = 1;
  get_set_go[10][7] = 1, get_set_go[10][8] = 1, get_set_go[11][7] = 1;
  get_set_go[11][8] = 1, get_set_go[12][7] = 1, get_set_go[12][8] = 1;
  get_set_go[13][5] = 1, get_set_go[13][6] = 1, get_set_go[13][7] = 1;
  get_set_go[13][8] = 1, get_set_go[13][9] = 1, get_set_go[13][10] = 1;
  get_set_go[14][5] = 1, get_set_go[14][6] = 1, get_set_go[14][7] = 1;
  get_set_go[14][8] = 1, get_set_go[14][9] = 1, get_set_go[14][10] = 1;
  show_message(get_set_go, false);
}
void game_over()
{
  memset(get_set_go, 0, sizeof get_set_go);
  get_set_go[1][1] = 1, get_set_go[1][2] = 1, get_set_go[1][4] = 1;
  get_set_go[1][7] = 1, get_set_go[1][11] = 1, get_set_go[1][13] = 1;
  get_set_go[1][14] = 1, get_set_go[1][15] = 1, get_set_go[2][0] = 1;
  get_set_go[2][3] = 1, get_set_go[2][5] = 1, get_set_go[2][7] = 1;
  get_set_go[2][8] = 1, get_set_go[2][10] = 1, get_set_go[2][11] = 1;
  get_set_go[2][13] = 1, get_set_go[3][0] = 1, get_set_go[3][3] = 1;
  get_set_go[3][4] = 1, get_set_go[3][5] = 1, get_set_go[3][7] = 1;
  get_set_go[3][9] = 1, get_set_go[3][11] = 1, get_set_go[3][13] = 1;
  get_set_go[3][14] = 1, get_set_go[4][0] = 1, get_set_go[4][2] = 1;
  get_set_go[4][3] = 1, get_set_go[4][5] = 1, get_set_go[4][7] = 1;
  get_set_go[4][11] = 1, get_set_go[4][13] = 1, get_set_go[5][1] = 1;
  get_set_go[5][2] = 1, get_set_go[5][3] = 1, get_set_go[5][5] = 1;
  get_set_go[5][7] = 1, get_set_go[5][11] = 1, get_set_go[5][13] = 1;
  get_set_go[5][14] = 1, get_set_go[5][15] = 1, get_set_go[9][1] = 1;
  get_set_go[9][2] = 1, get_set_go[9][5] = 1, get_set_go[9][8] = 1;
  get_set_go[9][9] = 1, get_set_go[9][10] = 1, get_set_go[9][11] = 1;
  get_set_go[9][13] = 1, get_set_go[9][14] = 1, get_set_go[9][15] = 1;
  get_set_go[10][0] = 1, get_set_go[10][3] = 1, get_set_go[10][5] = 1;
  get_set_go[10][8] = 1, get_set_go[10][9] = 1, get_set_go[10][13] = 1;
  get_set_go[10][15] = 1, get_set_go[11][0] = 1, get_set_go[11][3] = 1;
  get_set_go[11][5] = 1, get_set_go[11][8] = 1, get_set_go[11][9] = 1;
  get_set_go[11][10] = 1, get_set_go[11][13] = 1, get_set_go[11][14] = 1;
  get_set_go[11][15] = 1, get_set_go[12][0] = 1, get_set_go[12][3] = 1;
  get_set_go[12][5] = 1, get_set_go[12][8] = 1, get_set_go[12][9] = 1;
  get_set_go[12][13] = 1, get_set_go[12][14] = 1, get_set_go[13][1] = 1;
  get_set_go[13][2] = 1, get_set_go[13][6] = 1, get_set_go[13][7] = 1;
  get_set_go[13][9] = 1, get_set_go[13][10] = 1, get_set_go[13][11] = 1;
  get_set_go[13][13] = 1, get_set_go[13][15] = 1;
  show_message(get_set_go);
  reset(); // start all over
}

int main()
{
  disableJTAG();
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
  bool left_button, right_button;
  bool prev_pause_button = false;
  bool pause_button;
  bool paused = false;

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
      continue;

    // paddle movement
    if (left_button) // level trigger
      paddle.goleft();
    if (right_button)
      paddle.goright();

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
      if (ball.row > wall.down)
      {
        PORTA |= (1 << LIFE_BUTTON_PIN); // to sound the beeper
        _delay_ms(100);
        // PORTA &= ~(1 << LIFE_BUTTON_PIN); // to sound the beeper
        if (life)
        {
          life--;
          ball.init(); // reviving if life remains
        }
        else
          game_over();
      }

      ball_counter = 0;
    }

    bullets_counter++;
    if (bullets_counter >= bullet_speed)
    {
      update_bullets();
      bullets_counter = 0;
    }

    // Update life gain
    init_life_gain();
    catch_life();
    update_life_gain();

    // Update display buffer
    display.bricks_in_display();
    display.paddle_in_display();
    display.ball_in_display();
    display.bullet_in_display();
    display.life_gain_in_display();

    // sending to led driver
    display.send_to_driver();

    // turning off beeper
    PORTA &= ~(1 << SCORE_BUTTON_PIN);
    PORTA &= ~(1 << LIFE_BUTTON_PIN);
    PORTA &= ~(1 << RESET_BUTTON);
    PORTC &= ~(1 << LIFE_GAIN_PIN);

    // Adjusted delay for slower game execution
    _delay_ms(10);
  }

  return 0;
}
