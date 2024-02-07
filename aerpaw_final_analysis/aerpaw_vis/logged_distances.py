import re
import os

def extract_text_between_markers(filename):
    # Define the regex patterns for both sets of markers
    pattern_three = re.compile(r'(?<=\={3})(.*?)(?=\={3})')
    pattern_five = re.compile(r'(?<=\={5})(.*?)(?=\={5})')

    # Initialize lists to store extracted texts
    extracted_text_three = []
    extracted_text_five = []

    # Read the file and search for the patterns
    with open(filename, 'r', encoding='utf-8') as file:
        contents = file.read()

        # Find matches for three equal signs
        matches_three = pattern_three.findall(contents)
        extracted_text_three.extend(matches_three)

        # Find matches for five equal signs
        matches_five = pattern_five.findall(contents)
        extracted_text_five.extend(matches_five)

    return extracted_text_three, extracted_text_five


# the folder where the pickle file is located
# automagically search for the highest numbered folder within 'pickles/' directory
parent_folder = 'pickles/'
for i in range(0, 100): # check up to 100 folders
    if os.path.isdir(parent_folder + 'run' + str(i)):
        folder = parent_folder + 'run' + str(i) + '/'
print('folder: ', folder)

filename_suffix = 'log.txt'
# find all files with the suffix in the parent folder
files = [f for f in os.listdir(folder) if f.endswith(filename_suffix)]
# take the first file
filename = files[0]
print('filename: ', filename)

# extract the text between the markers
extracted_content_three, extracted_content_five = extract_text_between_markers(folder + filename)
print()

# Print the extracted content
print("Text between === and ===:")
for text in extracted_content_three:
    print(text)

print("\nText between ===== and =====:")
for text in extracted_content_five:
    print(text)
