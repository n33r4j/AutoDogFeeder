# Virtual Hardware Dashboard
# For simulation hardware using a GUI which you can interact with using a mouse.

import pygame
from VirtPIR import VirtPIR
from VirtLCD import VirtLCD
from VirtServo import VirtServo
import Colors



class VirtDashboard():
	def __init__(self):
		self.isRunning = True
		self.fps = 60
		self.screen_width = 800
		self.screen_height = 600
		
		
		pygame.init()
		
		self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
		pygame.display.set_caption("Virtual Dashboard")
		
		self.pir = VirtPIR(100,100,80)
		# TODO: PIR publisher/serice
		
		self.lcd = VirtLCD(300, 30)
		# TODO: LCD Subscriber

		self.servo1 = VirtServo(500, 250)
		self.servo1.setTarget(targets[0])
		# TODO: servo action server
		
		self.font = pygame.font.SysFont("arial", 20)
		self.open_label = self.font.render("OPEN", False, Colors.GREEN)
		self.close_label = self.font.render("CLOSE", False, Colors.RED)
		
		#self.servo2 = VirtServo(500, 450)
		#self.servo2.setTarget(45)
		
	def run(self):
		while self.isRunning:
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					self.isRunning = False
			
			self.screen.fill(Colors.LIGHT_GREY)
			
			message = ["       ~        ", 
					   "       ~        "]
			
			if not self.servo1.target_reached:
				self.servo1.update()
			elif targets:
				self.servo1.setTarget(targets[0])
				print(targets)
				targets.pop(0)
			
			if targets:
				message = [f"      {self.servo1.angle}      ", 
						   f" Target:{self.servo1.target_angle}    "]
			
			# Update sensors
			self.pir.update()
			
			if self.pir.motion_detected:
				message = ["     Motion     ", 
						   "   Triggered    "]

			self.lcd.setData(message)
			self.lcd.update(self.screen)
			
			#self.servo2.update(self.screen)
			self.pir.draw(self.screen)
			
			self.screen.blit(self.close_label, (self.servo1.x+25, self.servo1.y-50))
			self.screen.blit(self.open_label, (self.servo1.x-50, self.servo1.y+90))
			self.servo1.draw(self.screen)
			
			pygame.display.flip()
			pygame.time.Clock().tick(self.fps)
		
		pygame.quit()

if __name__ == "__main__":
	vd = VirtDashboard()
	vd.run()