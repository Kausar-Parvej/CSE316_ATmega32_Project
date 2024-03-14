//Ei Jinish Valoi Perar!

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>


// Define pins for LED matrix rows and columns
#define ROW_PORT PORTD
#define ROW_DDR DDRD
#define COL_PORT PORTB
#define COL_DDR DDRB

// Define pins for buttons
#define LEFT_BUTTON_PIN PA0
#define RIGHT_BUTTON_PIN PA1

// Function to initialize GPIO pins
void init_pins() {
	// Set LED matrix row pins as output
	ROW_DDR = 0xFF;
	// Set LED matrix column pins as output
	COL_DDR = 0xFF;
	// Enable internal pull-up resistors for button pins
	DDRA &= ~((1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN));
	PORTA |= (1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN);
}

// Function to read button inputs
unsigned int read_buttons() {
	return (PINA & ((1 << LEFT_BUTTON_PIN) | (1 << RIGHT_BUTTON_PIN)));
}

// Function to update LED matrix display
void update_display(unsigned int *display) {
	for (int row = 0; row < 8; row++) {
		// Set row pin low
		ROW_PORT = ~(1 << row);
		// Set column pins based on display data
		COL_PORT = display[row];
		// Delay to control display refresh rate
		_delay_ms(1);
	}
}

int main() {
	init_pins();

	// Initialize display buffer
	unsigned int display[8] = {0};

	// Ball position and direction
	int ball_row = 7;
	int ball_col = 0;
	int ball_dir_row = -1;
	int ball_dir_col = 1;

	int paddle_pos = 2;
	ball_col = paddle_pos+1;
	
	int ball_counter = 0; // for slowing down the ball movement :)
	int paddle_count = 0; // slowing down paddle movement
	int arr_index;
	
	int show_bricks[16] = {1,1,1,1,1,1,1,1,
						   1,1,1,1,1,1,1,1};

	// Main game loop
	while (1) {
		// Read button inputs
		unsigned int buttons = read_buttons();
		
		//paddle movement
		paddle_count ++;
		if(paddle_count >= 15){
			if (!(buttons & (1 << LEFT_BUTTON_PIN))) {
				if (paddle_pos > 0) {
					paddle_pos--;
				}
			}

			if (!(buttons & (1 << RIGHT_BUTTON_PIN))) {
				if (paddle_pos < 3) {
					paddle_pos++;
				}
			}
			paddle_count = 0;
		}

		// Update ball position
		ball_counter ++;
		if(ball_counter >= 15){
			ball_row += ball_dir_row;
			ball_col += ball_dir_col;
			
			// Check for collision with paddle
			
			/*if ((ball_row == 6 || ball_row == 7) && ball_col >= paddle_pos && ball_col < paddle_pos + 4) {
				ball_dir_row = -1; // Reflect ball upwards
			}*/
			
			if (ball_row == 6 || ball_row == 7) {   // for more refine collision
				if(ball_col == paddle_pos || ball_col == paddle_pos+1){
					ball_dir_row = -1;
					ball_dir_col = -1;
				}
				else if( ball_col == paddle_pos+2){
					ball_dir_row = -1;
					ball_dir_col = 0;
				}
				else if(ball_col == paddle_pos + 3 || ball_col == paddle_pos+4){
					ball_dir_row = -1;
					ball_dir_col = 1;
				}
			}
			
			// Check for collision with bricks
			if(ball_col != 0 && ball_col != 7){
				arr_index = ball_col + ball_dir_col;  // adding ball_dir_col for diagonal collision
			}
			else arr_index = ball_col;
			
			if(ball_row == 2 && ball_dir_row != 1){  // for now avoiding collision when coming down
				if(show_bricks[arr_index + 8]){
					show_bricks[arr_index + 8] = 0;  
					ball_dir_row = -ball_dir_row;
				}
			}
			if(ball_row == 1 && ball_dir_row != 1){
				if(show_bricks[arr_index]){
					show_bricks[arr_index] = 0;
					ball_dir_row = -ball_dir_row;
				}				
			}
			
			// Check for collision with walls
			if (ball_row <= 0 ) {
				ball_dir_row = -ball_dir_row; // Reflect ball downwards
			}
			if (ball_col <= 0 || ball_col >= 7) {
				ball_dir_col = -ball_dir_col; // Reflect ball horizontally
			}
			
			if(ball_row >= 15){ //  reviving the ball (for testing only!)
				ball_dir_row = -ball_dir_row;
				ball_row = 6;
				ball_col = paddle_pos+2;
			}
			
			
			ball_counter = 0;
		}
		


		// Update display buffer
		int led_row = 0;
		for(int i=0; i<2; i++){  // Show Bricks
			for(int j=0; j<8; j++){
				if(show_bricks[i*8 + j]){
					led_row |= (1<<j);
				}
			}
			display[i] = led_row;
			led_row = 0;
		}

		for (int i = 2; i <= 6; i++) {
			display[i] = 0; // Clear the middle rows
		}

		display[7] = (0b00011111 << paddle_pos); // Paddle (LSB is the left the of paddle)
		
		if(ball_row <= 7){
			display[ball_row] |= (1 << ball_col); // Ball
		}
		
		// Update display
		update_display(display);

		// Adjusted delay for slower game execution
		_delay_ms(5);
	}

	return 0;
}
