import tkinter as tk
from tkinter import ttk, messagebox
import serial
import time
import threading

class VoltageControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Voltage Control")

        self.create_widgets()

        self.running = False
        self.ser = None

    def create_widgets(self):
        # Create input fields
        self.dac_label = ttk.Label(self.root, text="DAC Output (ON/OFF):")
        self.dac_label.grid(row=0, column=0, padx=10, pady=5)
        self.dac_var = tk.StringVar(value="OFF")
        self.dac_entry = ttk.Combobox(self.root, textvariable=self.dac_var, values=["ON", "OFF"])
        self.dac_entry.grid(row=0, column=1, padx=10, pady=5)

        self.pwm_label = ttk.Label(self.root, text="PWM Output (ON/OFF):")
        self.pwm_label.grid(row=1, column=0, padx=10, pady=5)
        self.pwm_var = tk.StringVar(value="OFF")
        self.pwm_entry = ttk.Combobox(self.root, textvariable=self.pwm_var, values=["ON", "OFF"])
        self.pwm_entry.grid(row=1, column=1, padx=10, pady=5)

        self.v1_label = ttk.Label(self.root, text="Voltage on DAC output (V):")
        self.v1_label.grid(row=2, column=0, padx=10, pady=5)
        self.v1_entry = ttk.Entry(self.root)
        self.v1_entry.grid(row=2, column=1, padx=10, pady=5)

        self.v2_label = ttk.Label(self.root, text="Voltage on PWM output (V):")
        self.v2_label.grid(row=3, column=0, padx=10, pady=5)
        self.v2_entry = ttk.Entry(self.root)
        self.v2_entry.grid(row=3, column=1, padx=10, pady=5)

        self.delay_label = ttk.Label(self.root, text="Delay time (s):")
        self.delay_label.grid(row=4, column=0, padx=10, pady=5)
        self.delay_var = tk.StringVar(value="0.5")
        self.delay_entry = ttk.Entry(self.root, textvariable=self.delay_var)
        self.delay_entry.grid(row=4, column=1, padx=10, pady=5)

        # Create buttons
        self.start_button = ttk.Button(self.root, text="START", command=self.start)
        self.start_button.grid(row=5, column=0, padx=10, pady=10)

        self.stop_button = ttk.Button(self.root, text="STOP", command=self.stop)
        self.stop_button.grid(row=5, column=1, padx=10, pady=10)

        # Create text box for output
        self.output_text = tk.Text(self.root, height=10, width=50)
        self.output_text.grid(row=6, column=0, columnspan=2, padx=10, pady=10)

    def start(self):
        if not self.running:
            self.running = True
            self.output_text.delete(1.0, tk.END)  # Clear message window
            self.output_text.insert(tk.END, "Starting...\n")
            self.output_text.see(tk.END)
            try:
                self.ser = serial.Serial('COM3', 19200, timeout=1)
                time.sleep(0.5)
                self.thread = threading.Thread(target=self.run)
                self.thread.start()
            except serial.SerialException as e:
                messagebox.showerror("Error", f"Serial error: {e}")
                self.running = False

    def stop(self):
        if self.running:
            self.running = False
            self.output_text.insert(tk.END, "Stopping...\n")  # Clear message window
            self.output_text.see(tk.END)
            if self.ser:
                self.ser.close()

    def run(self):
        try:
            dacOUTPUT = self.dac_var.get()
            pwmOUTPUT = self.pwm_var.get()
            v1 = float(self.v1_entry.get()) if dacOUTPUT == 'ON' else 0.0
            v2 = float(self.v2_entry.get()) if pwmOUTPUT == 'ON' else 0.0
            delaytime = float(self.delay_var.get())

            # Debugging messages to check values
            print(
                f"DEBUG: dacOUTPUT = {dacOUTPUT}, pwmOUTPUT = {pwmOUTPUT}, v1 = {v1}, v2 = {v2}, delaytime = {delaytime}")

            if dacOUTPUT == 'ON' and abs(v1) > 9.8:
                self.output_text.insert(tk.END, "\nError: Your voltage for the DAC output is out of range\n")
                self.output_text.see(tk.END)
                self.stop()  # Stop the process
                return
            elif pwmOUTPUT == 'ON' and (v2 < 0.0 or v2 > 4.5):
                self.output_text.insert(tk.END, "\nError: Your voltage for the PWM output is out of range\n")
                self.output_text.see(tk.END)
                self.stop()  # Stop the process
                return

            v1_out, v2_out = self.Volt_outputs(dacOUTPUT, v1, pwmOUTPUT, v2, delaytime)
            self.output_text.delete(1.0, tk.END)  # Clear message window before showing new results
            if dacOUTPUT == 'ON':
                self.output_text.insert(tk.END, f"DAC output: {v1_out} V\n")
            if pwmOUTPUT == 'ON':
                self.output_text.insert(tk.END, f"PWM output: {v2_out} V\n")
            self.output_text.see(tk.END)

        except ValueError as e:
            self.output_text.insert(tk.END, f"Value error: {e}\n")
            self.output_text.see(tk.END)
        finally:
            self.running = False
            if self.ser:
                self.ser.close()

    def Volt_outputs(self, chan1, v1, chan2, v2, delaytime):
        # Format the data as a string
        data_to_send = f"{chan1} {v1} {chan2} {v2}\n"

        for i in data_to_send:
            time.sleep(delaytime)
            self.ser.write(i.encode())
        time.sleep(delaytime)

        response = self.ser.readline().decode().strip()
        print(f"DEBUG: Received response: {response}")  # Debug statement to check response
        if response:
            values = response.split()
            if len(values) == 2:
                v1_out, v2_out = values
                return float(v1_out), float(v2_out)
        return 0.0, 0.0


if __name__ == "__main__":
    root = tk.Tk()
    app = VoltageControlApp(root)
    root.mainloop()
