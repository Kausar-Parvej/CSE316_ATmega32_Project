// Ei Jinish Valoi Perar!

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>


// Define pins for LED matrix rows and columns
#define ROW_PORT PORTD
#define ROW_DDR DDRD
#define COL_PORT PORTB
#define COL_DDR DDRB

// Define pins for buttons
#define LEFT_BUTTON_PIN PA0
#define RIGHT_BUTTON_PIN PA1

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


//Maximum Bullet at a time
#define MAX_BULLETS 5
#define LEFT_SHOOT_PIN PD0
#define RIGHT_SHOOT_PIN PD1



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
	// Select MAX7219 module
	PORTB &= ~(1 << LOAD_PIN);

	// Send address and data
	SPI_send(address);
	SPI_send(data);
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
	DDRA = 0b11111100;

	//for Shooting bullets
	DDRD &= ~((1 << LEFT_SHOOT_PIN) | (1 << RIGHT_SHOOT_PIN));
    PORTD |= (1 << LEFT_SHOOT_PIN) | (1 << RIGHT_SHOOT_PIN);
}

// Function to update LED matrix display
void update_display(unsigned int *display)
{
	for (int row = 0; row < 8; row++)
	{
		// Set row pin low
		ROW_PORT = ~(1 << row);
		// Set column pins based on display data
		COL_PORT = display[row];
		// Delay to control display refresh rate
		_delay_ms(1);
	}
}

#define dimension 16

int bricks[dimension][dimension] = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
									{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
									{0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
									{0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
									{1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1},
									{0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0},
									{0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0},
									{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},

									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
									{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
								   };



struct Wall
{
	int left = 0;
	int right = 15;
	int up = 0;
	int down = 15;

	int center_col()
	{
		return (left + right) / 2;
	}
} wall;

struct Paddle
{
	int position; // position of the center of the paddle
	int size;
	int row;

	Paddle()
	{
		position = wall.center_col();
		size = 5;
		row = wall.down;
	}

	int left()
	{
		return position - size / 2;
	}
	int right()
	{
		return position + size / 2;
	}

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



//code for Shooting the bricks
int bullets_counter = 0;

// Define bullet structure
typedef struct {
    int row;
    int col;
    int active;
} Bullet;

// Array to store bullets
Bullet bullets[MAX_BULLETS];

bool prev_left_shoot = true, prev_right_shoot = true;
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
                bullets[i].col = paddle.position - 2; 
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
                bullets[i].col = paddle.position + 2; 
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
				bullets[i].active = 0;
			}
        }
    }
}





struct Ball
{
	int row, col;
	int row_dir, col_dir;

	void init()
	{
		row = wall.down+1;
		col = paddle.position;
		row_dir = -1;
		col_dir = 0;
	}

	Ball()
	{
		init();
	}

	int nextcol() { return col + col_dir ; }
	int nextrow() { return row + row_dir ; }
	void update()
	{
		row = nextrow();
		col = nextcol();
	}

	void collision_check()
	{
		// Check for collision with walls
		if (row <= wall.up)
			row_dir = 1; // Reflect ball downwards
		if (col <= wall.left || col >= wall.right)
			col_dir = -col_dir; // Reflect ball horizontally

		if (row >= wall.down + 7) //  reviving the ball (for testing only!)
		{
			init();
		}

		// Check for collision with paddle
		// if ((ball.nextrow() == paddle.row) && ball.col >= paddle.left() && ball.col <= paddle.right()) {
		//     ball.row_dir = -1; // Reflect ball upwards
		// }
		if (nextrow() == paddle.row && col >= paddle.left() && col <= paddle.right())
		{ // for more refine collision
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
		int rw = nextrow();
		int cl = nextcol();
		if ((rw>=0 && rw<=15) && bricks[rw][cl])
		{
			PORTA |= 0b00000100;
			bricks[rw][cl] = 0;
			row_dir = - row_dir;
			_delay_ms(5);
		}
	}
} ball;

struct Display {
	int rows = dimension;
	int cols = dimension;
	int splitrows = dimension/2;
	int splitcols = dimension/2;

	int ara[17][17];
	int split1[10];
	int split2[10];
	int split3[10];
	int split4[10]; // 10 for out_of_bound safety

	Display() {
		for (int i = 0; i < rows; ++i)
		for (int j = 0; j < cols; ++j)
		ara[i][j] = 0;
	}

	void bricks_in_display() {
		for (int i = 0; i < rows; ++i)
		for (int j = 0; j < cols; ++j)
		ara[i][j] = bricks[i][j];
	}
	void paddle_in_display() {
		for (int i = paddle.left(); i <= paddle.right(); ++i)
		ara[paddle.row][i] = 1;
	}
	void ball_in_display() {
		if (ball.row <= wall.down) ara[ball.row][ball.col] = 1;
	}

	void bullet_in_display() {
		for (int i = 0; i < MAX_BULLETS; i++) {
			if (bullets[i].active && bullets[i].row>=0) {
				ara[bullets[i].row][bullets[i].col] = 1;
			}
		}
	}

	void split() {
		// putting first 8*8 portion of display into split1
		for (int i = 0; i < splitrows; i++) {
			split1[i] = 0;
			for (int j = 0; j < splitcols; j++) {
				split1[i] |= (ara[i][j] << (splitcols - 1 - j)); // Using bitmasking to transform columns into a single integer
			}
		}
		// second portion to split2
		for (int i = 0; i < splitrows; i++) {
			split2[i] = 0;
			for (int j = cols - splitcols; j < cols; j++) {
				split2[i] |= (ara[i][j] << (cols - j - 1));
			}
		}
		// split3
		for (int i = rows - splitrows; i < rows; i++) {
			split3[i - rows + splitrows] = 0;
			for (int j = 0; j < splitcols; j++) {
				split3[i - rows + splitrows] |= (ara[i][j] << (splitcols - 1 - j));
			}
		}
		// last portion to split4
		for (int i = rows - splitrows; i < rows; i++) {
			split4[i - rows + splitrows] = 0;
			for (int j = cols - splitcols; j < cols; j++) {
				split4[i - rows + splitrows] |= (ara[i][j] << (cols - j - 1));
			}
		}
	}

	void send_to_driver() {
		for (uint8_t i = 1; i <= 8; i++)
		{
			// MAX7219_send2(i, 0b0001010); // Address i, data 0xFF (turn on all LEDs)// 4th matrix
			// MAX7219_send2(i, 0xaa);
			// MAX7219_send2(i, 0x11);
			// MAX7219_send2(i, 0x99); // 1st matrix
			MAX7219_send2(i, split4[i-1]);
			MAX7219_send2(i, split3[i-1]);
			MAX7219_send2(i, split2[i-1]);
			MAX7219_send2(i, split1[i-1]);

			deSelect_Load();
			// _delay_ms(100);
		}
		// _delay_ms(500);

		// Turn off all LEDs
		for (uint8_t i = 1; i <= 8; i++)
		{
			MAX7219_send(i, 0x00); // Address i, data 0x00 (turn off all LEDs)
		}
	}
} display;



int main()
{
	init_pins();

	// LED DRIVER FROM HERE
	// Initialize SPI communication
	SPI_init();

	// Set LOAD_PIN as output
	DDRB |= (1 << LOAD_PIN);
	// Set LOAD_PIN high (deSelect MAX7219 modules)
	PORTB |= (1 << LOAD_PIN);

	// Initialize MAX7219 drivers
	initMAX7219();
	// LED DRIVER ENDS HERE

	bool prev_left_button = false, prev_right_button = false;
	bool left_button, right_button;

	int ball_counter = 0; // for slowing down the ball movement

	GICR |= 1 << INT0;
	MCUCR |= 1 << ISC00;

	sei();

	// Initialize bullets
    for (int i = 0; i < MAX_BULLETS; i++) {
        bullets[i].active = 0; // Bullet is inactive initially
    }

	// Main game loop
	while (1)
	{
		// Read button inputs
		left_button = PINA & (1 << LEFT_BUTTON_PIN);
		right_button = PINA & (1 << RIGHT_BUTTON_PIN);

		// paddle movement
		if (!prev_left_button && left_button)
			paddle.goleft(); // raising edge
		if (!prev_right_button && right_button)
			paddle.goright();
		prev_left_button = left_button;
		prev_right_button = right_button;


		

		// Update ball position
		ball_counter++;
		if (ball_counter >= 5)
		{
			ball.update();

			ball.collision_check();

			ball_counter = 0;

		}

		bullets_counter++;
		if(bullets_counter >= 3){
			update_bullets();
			bullets_counter = 0;
		}

		
		

		// Update display buffer
		display.bricks_in_display();

		// for (int i = 2; i <= 6; i++)
		// {
		//   display[i] = 0; // Clear the middle rows
		// }

		// display[7] = (0b00011111 << paddle.left()); // Paddle (LSB is the left the of paddle)
		display.paddle_in_display();

		// if (ball.row <= wall.down)
		// {
		//   display[ball.row] |= (1 << ball.col); // Ball
		// }
		display.ball_in_display();
		display.bullet_in_display();

		// Update display
		// update_display(display);
		display.split();

		display.send_to_driver();

		// Adjusted delay for slower game execution
		_delay_ms(5);
		PORTA &= ~(0b00000100);
	}

	return 0;
}

ISR(INT0_vect)
{
	if (i == 0)
	{
		TCCR1B |= 1 << CS10;
		i = 1;
	}
	else
	{
		TCCR1B = 0;
		pulse = TCNT1;
		TCNT1 = 0;
		i = 0;
	}
}