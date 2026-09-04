import queue
import threading

class Receiver:
	def __init__(self):
		self.command_queue = queue.Queue()
		threading.Thread(target=self._read_commands, daemon=True).start()

	def _read_commands(self):
		print("Enter set_target_velocity <value> or set_target_x_velocity <value>")
		while True:
			try:
				self.command_queue.put(input("> ").split())
			except EOFError:
				break

	def receive(self, target_y_velocity, target_x_velocity):
		changed = False

		while True:
			try:
				command_parts = self.command_queue.get_nowait()
			except queue.Empty:
				break

			if len(command_parts) != 2:
				print("Use: set_target_velocity <value> or set_target_x_velocity <value>")
				continue

			command, value = command_parts
			try:
				if command not in ("set_target_velocity", "set_target_x_velocity"):
					print("Unknown command. Use set_target_velocity or set_target_x_velocity.")
					continue
				new_velocity = float(value)
			except (ValueError, TypeError):
				print("Use a numeric value, for example: set_target_x_velocity 0")
				continue

			if command == "set_target_velocity":
				target_y_velocity = new_velocity
			else:
				target_x_velocity = new_velocity

			changed = True
			print(
				f"Target x velocity = {target_x_velocity}, "
				f"target y velocity = {target_y_velocity}"
			)

		return target_y_velocity, target_x_velocity, changed
