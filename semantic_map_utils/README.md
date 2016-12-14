# Semantic Map Utils

this package contains a collection of nodes that can be used to simplify the task of working with the semantic map.

## Insert a new map

To insert a new map, just run the the `insert_csv.py` script. The help function should explain all the features:

```
$ rosrun semantic_map_utils insert_csv.py -h
usage: insert_csv.py [-h] -i INPUT [--db_name DB_NAME]
                     [--collection_name COLLECTION_NAME] [--db_host DB_HOST]
                     [--db_port DB_PORT]
                     dataset_name

positional arguments:
  dataset_name          The name of the dataset. Saved in meta information
                        using 'meta_name'

optional arguments:
  -h, --help            show this help message and exit
  -i INPUT, --input INPUT
                        Input yaml file
  --db_name DB_NAME     The database name. Default: semantic_map
  --collection_name COLLECTION_NAME
                        The collection name. Default: idea_park
  --db_host DB_HOST     The database host address. Default: localhost
  --db_port DB_PORT     The database port. Default: 62345

```

An example file can be found in the examples folder of this package. Example usage would be:

```
rosrun semantic_map_utils insert_csv.py -i $(rospack find semantic_map_utils)/examples/example_map.csv my_map
```

Which inserts the data in the file into the mongodb under the name `my_map`.

_If you change any of the other parameters, you will have to make sure that all the started components are notfied because all of them assume the default values being true._

## Creating new waypoints

The script `save_current_location.py` saves the current location of the robot to the mongodb as a waypoint. this requires the `semantic_map_transform_publisher` to be running. The only parameter that is essential is the waypoint name:

```
rosrun semantic_map_utils save_current_location.py _waypoint_name:=waypoint1
```

This will save the current location of the robot under the name `waypoint1`. Please, do only use lower case letters and start the name with a letter due to the limitations of the planner.

Other parameters are:

* `db_name` _Default: `semantic_map`_: The name of the database to save the waypoints in.
* `collection_name` _Default: `waypoints`_: The name of the cololection inside the database to save the waypoints in.
* `target_frame` _Default: `semantic_map`_: The name of the parent tf frame.

_Please, only change the default values if you know what you're doing ;)_