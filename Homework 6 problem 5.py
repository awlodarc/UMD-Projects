import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.special import erfc

# Number of transmissions for BER calculation
num_trials = 110000
# SNR range from 0 to 10 dB
snr_db_range = np.arange(0, 10, 0.25)
ber_bpsk = []
ber_qpsk = []
ber_16qam = []
ber_bpsk_theoretical = []
ber_qpsk_theoretical = []
ber_16qam_theoretical = []

# Define QPSK constellation points with Gray encoding (aligned to the diagram)
qpsk_constellation = {
    (0, 0): (-1 - 1j) / np.sqrt(2),
    (0, 1): (-1 + 1j) / np.sqrt(2),
    (1, 0): (1 - 1j) / np.sqrt(2),
    (1, 1): (1 + 1j) / np.sqrt(2)
}

# Define 16QAM constellation points with Gray encoding
qam16_constellation = {
    (0, 0, 0, 0): (-3 + 3j) / np.sqrt(10),
    (0, 0, 0, 1): (-1 + 3j) / np.sqrt(10),
    (0, 0, 1, 1): (1 + 3j) / np.sqrt(10),
    (0, 0, 1, 0): (3 + 3j) / np.sqrt(10),
    (0, 1, 0, 0): (-3 + 1j) / np.sqrt(10),
    (0, 1, 0, 1): (-1 + 1j) / np.sqrt(10),
    (0, 1, 1, 1): (1 + 1j) / np.sqrt(10),
    (0, 1, 1, 0): (3 + 1j) / np.sqrt(10),
    (1, 1, 0, 0): (-3 - 1j) / np.sqrt(10),
    (1, 1, 0, 1): (-1 - 1j) / np.sqrt(10),
    (1, 1, 1, 1): (1 - 1j) / np.sqrt(10),
    (1, 1, 1, 0): (3 - 1j) / np.sqrt(10),
    (1, 0, 0, 0): (-3 - 3j) / np.sqrt(10),
    (1, 0, 0, 1): (-1 - 3j) / np.sqrt(10),
    (1, 0, 1, 1): (1 - 3j) / np.sqrt(10),
    (1, 0, 1, 0): (3 - 3j) / np.sqrt(10)
}

# Loop over each SNR level
for snr_db in snr_db_range:
    # Convert SNR from dB to linear scale
    snr_linear = 10 ** (snr_db / 10)
    # Finding varience for each SNR level
    noise_variance = 1 / (2 * snr_linear)

    # Initialize error counters for each SNR level
    errors_bpsk = 0
    errors_qpsk = 0
    errors_16qam = 0

    # Run trials for BER calculation at current SNR level
    for _ in range(num_trials):
        # Generate noise for each transmission
        noise_bpsk = np.random.normal(0, np.sqrt(noise_variance))

        noise_qpsk_real = np.random.normal(0, np.sqrt(noise_variance / 2))
        noise_qpsk_imag = np.random.normal(0, np.sqrt(noise_variance / 2))
        noise_qpsk = noise_qpsk_real + 1j * noise_qpsk_imag

        noise_16qam_real = np.random.normal(0, np.sqrt(noise_variance / 2))
        noise_16qam_imag = np.random.normal(0, np.sqrt(noise_variance / 2))
        noise_16qam = noise_16qam_real + 1j * noise_16qam_imag

        # Generate random bits for BPSK, QPSK, and 16QAM
        rand_bits_bpsk = random.getrandbits(1)
        rand_bits_qpsk = random.getrandbits(2)
        rand_bits_16qam = random.getrandbits(4)

        # Extract individual bits for QPSK and 16QAM
        qpsk_msb = (rand_bits_qpsk >> 1) & 1
        qpsk_lsb = rand_bits_qpsk & 1

        _16qam_bits = []

        for i in reversed(range(4)):
            bit = (rand_bits_16qam >> i) & 1
            _16qam_bits.append(bit)

        # Map bits to constellation points
        if rand_bits_bpsk == 1:
            bpsk_symbol = 1
        else:
            bpsk_symbol = -1
        
        qpsk_symbol = qpsk_constellation[(qpsk_msb, qpsk_lsb)]
        _16qam_symbol = qam16_constellation[tuple(_16qam_bits)]

        # Add AWGN noise
        received_bpsk = bpsk_symbol + noise_bpsk
        received_qpsk = qpsk_symbol + noise_qpsk
        received_16qam = _16qam_symbol + noise_16qam

        # BPSK Slicer
        if received_bpsk >= 0:
            decoded_bit_bpsk = 1
        else:
            decoded_bit_bpsk = 0
        if decoded_bit_bpsk != rand_bits_bpsk:
            errors_bpsk += 1

        # QPSK Slicer
        min_distance = float('inf')
        decoded_bits_qpsk = (0, 0)
        for bits, symbol in qpsk_constellation.items():
            distance = abs(received_qpsk - symbol)
            if distance < min_distance:
                min_distance = distance
                decoded_bits_qpsk = bits
        if decoded_bits_qpsk != (qpsk_msb, qpsk_lsb):
            errors_qpsk += (decoded_bits_qpsk[0] != qpsk_msb) + (decoded_bits_qpsk[1] != qpsk_lsb)

        # 16QAM Slicer
        min_distance = float('inf')
        decoded_bits_16qam = (0, 0, 0, 0)
        for bits, symbol in qam16_constellation.items():
            distance = abs(received_16qam - symbol)
            if distance < min_distance:
                min_distance = distance
                decoded_bits_16qam = bits
        if decoded_bits_16qam != tuple(_16qam_bits):
            errors_16qam += sum(decoded_bits_16qam[i] != _16qam_bits[i] for i in range(4))

    # Calculate BER for this SNR level
    ber_bpsk.append(errors_bpsk / num_trials)
    ber_qpsk.append(errors_qpsk / (2 * num_trials))
    ber_16qam.append(errors_16qam / (4 * num_trials))

    # Calculate theoretical BER for each modulation
    ber_bpsk_theoretical.append(0.5 * erfc(np.sqrt(2 * snr_linear)/(np.sqrt(2))))
    ber_qpsk_theoretical.append(0.5 * erfc(np.sqrt(2 * snr_linear)/(np.sqrt(2))))
    ber_16qam_theoretical.append(3/4 * 0.5 * erfc(np.sqrt((4 * snr_linear) / 5)/(np.sqrt(2))))

# Plot for BPSK
plt.figure()
plt.semilogy(snr_db_range, ber_bpsk, label='BPSK Simulated')
plt.semilogy(snr_db_range, ber_bpsk_theoretical, label='BPSK Theoretical')
plt.xlabel('SNR (dB)')
plt.ylabel('BER')
plt.title('BER vs. SNR for BPSK')
plt.legend()
plt.grid()

# Plot for QPSK
plt.figure()
plt.semilogy(snr_db_range, ber_qpsk, label='QPSK Simulated')
plt.semilogy(snr_db_range, ber_qpsk_theoretical, label='QPSK Theoretical')
plt.xlabel('SNR (dB)')
plt.ylabel('BER')
plt.title('BER vs. SNR for QPSK')
plt.legend()
plt.grid()

# Plot for 16QAM
plt.figure()
plt.semilogy(snr_db_range, ber_16qam, label='16QAM Simulated')
plt.semilogy(snr_db_range, ber_16qam_theoretical, label='16QAM Theoretical')
plt.xlabel('SNR (dB)')
plt.ylabel('BER')
plt.title('BER vs. SNR for 16QAM')
plt.legend()
plt.grid()

plt.show()
