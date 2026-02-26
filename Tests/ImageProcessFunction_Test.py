import numpy as np
import cv2 as cv
from src import processImage_Function

THRESH = 0.80  # <-- single source of truth

file1 = 'TestImages/TestImg7.JPG'
file2 = 'TestImages/TestImg8.JPG'
file3 = 'TestImages/TestImg9.JPG'

img1 = cv.imread(file1)
img2 = cv.imread(file2)
img3 = cv.imread(file3)

results = []
processedImages = {}

def run_one(img, filename, label):
    if img is None:
        print(f"Error loading {filename}")
        return

    print(f"Processing {filename}...")
    # IMPORTANT: pass the same threshold the test uses
    drawn_img, contour, score, origin, corners = processImage_Function.processImage(img, rectThreshold=THRESH)

    if score is not None:
        print(f"- Score: {score:.4f}")

        if score < THRESH:
            print(f"- WARNING: Contour quality is below the {THRESH:.2f} standard.")

        results.append({'name': label, 'contour': contour, 'score': score, 'origin': origin, 'corners': corners})
        processedImages[label] = drawn_img
        print(f"- Added {filename} to results list.")
    else:
        print(f"- No valid contour found in {filename}.")

run_one(img1, file1, 'Image 1 (Altitude 1)')
run_one(img2, file2, 'Image 2 (Altitude 2)')
run_one(img3, file3, 'Image 3 (Altitude 3)')

if not results:
    print("No valid panels were detected in any image.")
else:
    validResults = [r for r in results if r['score'] is not None]
    if not validResults:
        print("No valid rectangularity scores found.")
    else:
        bestResult = max(validResults, key=lambda r: r['score'])

        if bestResult['contour'] is not None and bestResult['score'] >= THRESH:
            bestName = bestResult['name']
            bestOrigin = bestResult.get('origin', None)

            print(f"--- Best panel found in: {bestName} ---")
            print(f"Rectangularity Score: {bestResult['score']:.4f}")

            if bestOrigin is not None:
                print(f"\n>>> Panel origin (Top-Left Corner): {bestOrigin} <<<")
            else:
                print("\n>>> Panel origin (Top-Left Corner): Not found <<<")

            print("\nDisplaying images:")
            for name, img in processedImages.items():
                print(f"Showing window: {name}")
                cv.imshow(name, img)

            cv.imshow(f"Best Result: {bestName}", processedImages[bestName])
        else:
            print(f"\nNo panel met the quality threshold (score >= {THRESH:.2f})")
            for name, img in processedImages.items():
                cv.imshow(name, img)

print("\nWaiting for key press...")
cv.waitKey(0)
cv.destroyAllWindows()
