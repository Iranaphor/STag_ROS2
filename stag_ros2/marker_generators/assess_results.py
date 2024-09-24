import os
import csv
from PIL import Image, ImageDraw
from collections import Counter

base = 21
base = int(os.getenv('MS'))

total_masks = 100

sample_size = 10
if base == 23:
    sample_size = 6


N = sample_size * total_masks

print(f'For Markers with Base of HD{base}')
csv_file = f"{os.getenv('HOME')}/STag-Markers/occlusions/setF/{base}/results.csv"
img_file = f"{os.getenv('HOME')}/STag-Markers/standard/HD{base}/00000.png"
img_outfile = f"{os.getenv('HOME')}/STag-Markers/occlusions/setF/{base}/results"


# List to store the processed values from the last column
processed_values = []
processed_coords = []

# Read the CSV file
first=True
headings = dict()
with open(csv_file, 'r', newline='', encoding='utf-8') as file:
    reader = csv.reader(file)
    for row in reader:

        # Skip first row
        if first:
            headings = {r:i for i,r in enumerate(row)}
            first = False
            continue

        # Get the last column value
        detected_value = row[headings['detected_value']].strip()
        input_png = row[headings['marker_file']].strip()

        # Set value to False if it's 'X', else True
        if detected_value.upper() == 'X':
            detected_value = -1
        if int(detected_value) == int(input_png.replace('.png','').replace(f'HO{base}-ID','')):
            processed_values.append(True)
        else:
            processed_values.append(False)
        processed_coords.append([row[headings['x']].strip(), row[headings['y']].strip()])

# Ensure we have at least 2000 entries
if len(processed_values) < (2*N):
    raise ValueError(f"The CSV file must contain at least {N} rows.")

# Get the first 1000 entries and the next 1000 entries
first_N = processed_values[:N]
next_N = processed_values[N:(2*N)]


# Count the permutations
permutation_counts = Counter()

for val1, val2 in zip(first_N, next_N):
    permutation = (val1, val2)
    permutation_counts[permutation] += 1

# Map permutations to their string representations for output
permutation_labels = {
    (True, True): 'True True',
    (True, False): 'True False',
    (False, True): 'False True',
    (False, False): 'False False'
}

# Print the counts
print('-----')
for permutation, count in permutation_counts.items():
    label = permutation_labels[permutation]
    print(f"{label}: {count}")
print('-----')


from statsmodels.stats.contingency_tables import mcnemar

# Replace these with your actual counts
n11 = permutation_counts[(True, True)] # Both systems succeeded
n10 = permutation_counts[(True, False)] # Baseline succeeded, Adapted failed
n01 = permutation_counts[(False, True)] # Baseline failed, Adapted succeeded
n00 = permutation_counts[(False, False)] # Both systems failed

table = [[n11, n10],
         [n01, n00]]

result = mcnemar(table, exact=True)
print('P-Value:', result.pvalue)

hd_accuracy = (n11+n10) / (n11+n10+n01+n00)
print('HD Accuracy:', hd_accuracy)

ho_accuracy = (n11+n01) / (n11+n10+n01+n00)
print('HO Accuracy:', ho_accuracy)




# Load image to plot occlusions to
img_TT = Image.open(img_file)
img_TF = Image.open(img_file)
img_FT = Image.open(img_file)
img_FF = Image.open(img_file)
img_IND = Image.open(img_file)
draw = ImageDraw.Draw(img_IND)

permutation_imgs = {
    (True,True): ImageDraw.Draw(img_TT),
    (True,False): ImageDraw.Draw(img_TF),
    (False,True): ImageDraw.Draw(img_FT),
    (False,False): ImageDraw.Draw(img_FF)
}

first_N_coords = processed_coords[:N]
next_N_coords = processed_coords[N:(2*N)]
cross_size=20
for val1, val2, xy1, _ in zip(first_N, next_N, first_N_coords, next_N_coords):

    # Identify permutation and perpare coordinates
    permutation = (val1, val2)
    x,y = int(xy1[0]), int(xy1[1])

    # Define labelling colour for markings
    if val1 and val2: fill='red'
    elif val1 and not val2: fill='blue'
    elif not val1 and val2: fill='green'
    elif not val1 and not val2: fill='purple'

    # Apply cross to appropriate permutation graph
    permutation_imgs[permutation].line((x - cross_size, y, x + cross_size, y), fill=fill, width=2)
    permutation_imgs[permutation].line((x, y - cross_size, x, y + cross_size), fill=fill, width=2)

    # Apply cross to appropriate contingency graph
    draw.line((x - cross_size, y, x + cross_size, y), fill=fill, width=2)
    draw.line((x, y - cross_size, x, y + cross_size), fill=fill, width=2)



img_TT.save(img_outfile+'_TT.png')
img_TF.save(img_outfile+'_TF.png')
img_FT.save(img_outfile+'_FT.png')
img_FF.save(img_outfile+'_FF.png')
img_IND.save(img_outfile+'_IND.png')




# Determine average as robustness for a sampled marker against occlusions
data={'HD':dict(), 'HO':dict()}
first=True
headings = dict()

with open(csv_file, 'r', newline='', encoding='utf-8') as file:
    reader = csv.reader(file)
    for row in reader:

        # Skip first row
        if first:
            headings = {r:i for i,r in enumerate(row)}
            first = False
            continue

        # Get the last column value
        detected_value = row[headings['detected_value']].strip()
        input_png = row[headings['marker_file']].strip()
        typ = 'HO' if 'HO' in input_png else 'HD'
        id = input_png.replace('.png','').replace(f'HO{base}-ID','')
        if id not in data[typ]:
            data[typ][id] = 0.0
            data[typ][id+"x"] = []
        if detected_value == 'x':
            data[typ][id] += 0
            data[typ][id+"x"] += [0]
        elif int(id) == int(detected_value):
            data[typ][id] += 1
            data[typ][id+"x"] += [1]


import numpy as np
for group, typ in data.items():
    i=[]
    for key, marker in typ.items():
        if 'x' in key:
            #print(f"{group}{base}", key, f"Detections:{sum(marker)}", f"Failures:{len(marker)-sum(marker)}")
            pass
        else:
            #print(f"{group}{base}", key, f"{(round(marker*100,3))}%")
            i += [marker]

    print(f"{group}{base}", f"Average Detections: {round(np.average(np.array(i)),3)}, Std: {round(np.std(np.array(i)),3)}")
