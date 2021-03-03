# DroneDisturbance
Data and code for drone disturbance paper

## Files
### supplement folder
The supplement folder contains the raw data files and code used to generate the dataframes used in the analyses in the main text.
- data-cleaning.ipynb – Jupyter notebook containing the code required to clean the raw drone flight logs and raw wind data (scraped from Airdata.com). This generates the cleaned files found in the clean-data folder. The code in this notebook does not need to be run if the cleaned files are in clean-data/drone-logs/ and clean-data/wind-data/.
- data_cleaning_functions.py – script containing functions used in data-cleaning.ipynb
