
#-----------------GETTING LIMELIGHT DATA IN JSON
import requests
import json

# The API endpoint
url = "http://192.168.0.102:5807/results"
# url = "https://jsonplaceholder.typicode.com/posts/1"

# A GET request to the API
response = requests.get(url)

# Print the response
response_json = response.json()
# print(response_json['id']+response_json['userId'])
print(response_json)
print("")
for i in response_json['Results']['Retro']:
    print(i)
    print("")

print("")
print(response_json['Results']['Retro'][0]['ta'])