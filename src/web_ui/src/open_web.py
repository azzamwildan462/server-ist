from selenium import webdriver
import time

# Use localhost URL
url = "http://localhost:8000"

options = webdriver.ChromeOptions()
options.add_argument("--start-fullscreen")
options.add_argument("--autoplay-policy=no-user-gesture-required")

driver = webdriver.Chrome(options=options)
driver.get(url)

print(f"Opened {url} in Chrome browser (fullscreen).")

while True:
    time.sleep(3600)
