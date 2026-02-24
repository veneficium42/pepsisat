import numpy as np
from PIL import Image

size = 500
eps = 1e-9

window_1d = np.hamming(size)
window_2d = np.sqrt(np.outer(window_1d, window_1d))

img1 = Image.open("image1.png").crop((0, 0, size, size)).convert("L")
img2 = Image.open("image2.png").crop((0, 0, size, size)).convert("L")

arr1 = np.array(img1, dtype=np.float32) * window_2d
arr2 = np.array(img2, dtype=np.float32) * window_2d

F1 = np.fft.fft2(arr1)
F2 = np.fft.fft2(arr2).conjugate()

R = (F1 * F2) / (np.abs(F1 * F2) + eps)

corr = np.abs(np.fft.ifft2(R))
corr = np.fft.fftshift(corr)

corr -= corr.min()
corr /= corr.max()
corr = (corr * 255).astype(np.uint8)

Image.fromarray(corr, mode="L").save("correlation.png")
