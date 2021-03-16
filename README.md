# DroneDisturbance
Data and code for drone disturbance paper

## Files
### data folder
### supplement folder
The supplement folder contains the raw data files and code used to generate the dataframes used in the analyses in the main text.
- data_cleaning_functions.py – script containing functions used in data-cleaning.ipynb
- data_processing_functions.py -
- data-cleaning.ipynb – Jupyter notebook containing the code required to clean the raw drone flight logs and raw wind data (scraped from Airdata.com). This generates the cleaned files found in the clean-data folder. The code in this notebook does not need to be run if the cleaned files are in clean-data/drone-logs/ and clean-data/wind-data/.
- data-processing.ipynb -
- interobserver-reliability.ipynb -
#### clean-data folder
#### processed-data folder
#### raw-data folder
- exchanges.csv -
- first_clips.csv -
- repeatability.csv -
- whole_observation.csv -
##### drone-flight-files folder
This folder contains the logs for each drone flight, which were downloaded from the DJI Go 4 app.
##### wind-data-csv folder
This folder contains the in-flight wind data for each flight. The data were scraped from Airdata.com, which calculates the wind speed during drone flights based on the motor outputs of the drone.
