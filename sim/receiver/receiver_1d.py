import queue
import threading


class Receiver:
	def __init__(self):
		self.command_queue = queue.Queue()
		threading.Thread(target=self._read_commands, daemon=True).start()

	def _read_commands(self):
		print("Enter set_target_velocity <value>")
		while True:
			try:
				self.command_queue.put(input("> ").split())
			except EOFError:
				break

	def receive(self, estimated_height, target_height, target_velocity):
		changed = False

		while True:
			try:
				command_parts = self.command_queue.get_nowait()
			except queue.Empty:
				break

			if len(command_parts) != 2:
				print("Use: set_target_velocity <value>")
				continue

			command, value = command_parts
			try:
				if command != "set_target_velocity":
					print("Unknown command. Use set_target_velocity.")
					continue
				target_velocity = float(value)
			except (ValueError, TypeError):
				print("Use a numeric value, for example: set_target_velocity 0")
				continue

			if target_velocity == 0:
				target_height = estimated_height

			changed = True
			print(f"Target height = {target_height}, target velocity = {target_velocity}")

		return target_height, target_velocity, changed
