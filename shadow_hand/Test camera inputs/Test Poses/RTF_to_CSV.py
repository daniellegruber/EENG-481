import csv
import re
from striprtf.striprtf import rtf_to_text

def extract_data_from_rtf(rtf_file):
    # Read and extract plain text from RTF
    with open(rtf_file, 'r', encoding='utf-8') as file:
        rtf_content = file.read()
        text = rtf_to_text(rtf_content)  # Convert RTF to plain text
    
    # Regular expression to match the pattern "Category Name, Value;"
    pattern = re.findall(r'([A-Za-z]+\s[A-Z]+),\s([\d\.]+);', text)
    
    return pattern

def write_to_csv(data, output_csv):
    # Write extracted data to CSV
    with open(output_csv, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Category", "Value"])  # Header
        csv_writer.writerows(data)

if __name__ == "__main__":
    input_rtf = "Pair 8/Untitled 2.rtf"
    output_csv = "Pair 8/output.csv"
    
    extracted_data = extract_data_from_rtf(input_rtf)
    write_to_csv(extracted_data, output_csv)
    
    print(f"CSV file '{output_csv}' has been created successfully.")
