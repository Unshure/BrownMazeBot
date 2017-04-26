import serial
import pygame
import sys

# Open arduino serial port with default baud rate (9600)
arduino = serial.Serial('/dev/ttyACM0')

# Pygame ish
bkg = (255, 255, 255)
clr = (0, 0, 0)
pygame.init()
size = (200, 200)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Robot Controller")
pygame.key.set_repeat(1, 1)
font = pygame.font.SysFont("Stencil", 20)
clock = pygame.time.Clock()

# Initial value of speed
speed = 200
dev_id = 0

# Loop to capture keystrokes
while True:
	# Check for quit && increase/decrease speed
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			arduino.write('[0 0]\n');
			pygame.quit()
			sys.exit()
		elif event.type == pygame.KEYUP:
			if event.key == pygame.K_a:
				speed += 10
			elif event.key == pygame.K_s:
				speed -= 10

			if event.key == pygame.K_z:
				dev_id += 1
			elif event.key == pygame.K_x:
				dev_id -= 1

	if dev_id < 0:
		dev_id = 0
	elif dev_id > 10:
		dev_id = 10

	# Reset info text & action
	txt = ""
	action = ""

	# Grab keystrokes
	keys_pressed = pygame.key.get_pressed()

	if keys_pressed[pygame.K_q]:
		pygame.quit()
		sys.exit()
	elif keys_pressed[pygame.K_UP]:
		txt = "Forward"
		action = 'F'
	elif keys_pressed[pygame.K_DOWN]:
		txt = "Backward"
		action = 'B'
	elif keys_pressed[pygame.K_LEFT]:
		txt = "Left point turn"
		action = 'L'
	elif keys_pressed[pygame.K_RIGHT]:
		txt = "Right point turn"
		action = 'R'

	# Send values over arduino
        buf = '[' + action + ' ' + 1 + ']\n'
	print('SERIAL WRITE to /dev/ttyACM0 : ' + buf)
	flushinput();
	flushoutput();
	flush();
	arduino.write(buf)

	# More pygame ish
	screen.fill(bkg)
	speed_text = font.render('Speed: ' + str(speed), True, clr)
	screen.blit(speed_text, [10, 10])
	dev_id_text = font.render('DEV_ID: ' + str(dev_id), True, clr)
	screen.blit(dev_id_text, [10, 30])
	info_text = font.render(txt, True, clr)
	screen.blit(info_text, [10, 50])
	pygame.display.flip()
	clock.tick(60)
