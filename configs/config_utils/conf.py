import json
from json_minify import json_minify
class Conf:
    def __init__(self, confPath):
        """
        load and store the configuration and update the 
        object't dictinonary
        """
        conf = json.loads(json_minify(open(confPath).read()))
        self.__dict__.update(conf)
    def __getitem__(self,k):
            # return the value associate with the supplied key
            return self.__dict__.get(k, None)
                  
if __name__ == '__main__':

  print(test_config["max_cosine_distance"])
  print(test_config_2["max_euclidean_distance"])
  print(test_config_3["model_path"])
  line_conf = {"x":100,"y":200}
  
  
  with open('rules/detection_rule.json', 'w') as outfile:
    json.dump(line_conf, outfile)
  