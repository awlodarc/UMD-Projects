import numpy as np
import matplotlib.pyplot as plt

sampling_rate = 1000  # Samples per second
T = 1.0 / sampling_rate  # Sampling period
t = np.arange(0, 1, T)  # Time vector for 1 second

frequency = 5  # Frequency of the sine wave in Hz
amplitude = 1  # Amplitude of the sine wave
signal = amplitude * np.sin(2 * np.pi * frequency * t)

N = len(signal)
dft_result = np.zeros(N, dtype='complex128')  # Initialize the result as complex numbers

for k in range(N):
    dft_result[k] = np.sum(signal * np.exp(-1j * 2 * np.pi * k * np.arange(N) / N))

# Calculate the magnitude of the DFT
dft_magnitude = np.abs(dft_result)

# Manually calculate the corresponding frequency values
frequencies = np.array([k / (N * T) for k in range(N)])

# Plot the time domain signal
plt.subplot(2, 1, 1)
plt.plot(t, signal)
plt.title("Time Domain Signal (Sine Wave, 5 Hz)")
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")

# Plot the frequency domain (DFT result)
plt.subplot(2, 1, 2)
plt.plot(frequencies, dft_magnitude)  # Plot only the positive frequencies
plt.title("Frequency Domain (Manual DFT)")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")

plt.tight_layout()
plt.show()
