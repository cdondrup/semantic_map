#!/usr/bin/env python

from pymongo import MongoClient
import yaml
import argparse
import sys


def file_reader(inputfile):
    print "Opening %s" % inputfile
    with open(inputfile) as f:
        return yaml.load(f)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_name", help="The name of the dataset. Saved in meta information using 'meta_name'", type=str)
    parser.add_argument("-i", "--input", help="Input yaml file", type=str, required=True)
    parser.add_argument("--db_name", help="The database name. Default: semantic_map", type=str, default="semantic_map")
    parser.add_argument("--collection_name", help="The collection name. Default: idea_park", type=str, default="idea_park")
    parser.add_argument("--db_host", help="The database host address. Default: localhost", type=str, default="localhost")
    parser.add_argument("--db_port", help="The database port. Default: 62345", type=str, default="62345")
    args = parser.parse_args()
    
    try:
        client = MongoClient(args.db_host, int(args.db_port))
    except ValueError as e:
        print "'%s' is not a valid port number." % args.db_port
        print e
        sys.exit(1)
    
    db = client[args.db_name]   
    if db[args.collection_name].find_one({"semantic_map_name": args.dataset_name}) is not None:
        print "The database already contains a semantic map with the name '%s'." % args.dataset_name
        print "Override current entry?"
        answer = raw_input("[y/n]: ")
        if answer.lower() == "n":
            print "Aborting database insertion."
            sys.exit(1)
        elif answer.lower() == "y":
            print "Removing old entries."
            db[args.collection_name].remove({"semantic_map_name": args.dataset_name})
        else:
            print "Unknown option '%s'" % answer
            sys.exit(1)

    for shop in file_reader(args.input)["shops"]:
        shop["semantic_map_name"] = args.dataset_name
        db[args.collection_name].insert(shop)

    print "Inserted new entries as '%s'." % args.dataset_name

    db[args.collection_name].ensure_index("id")
    db[args.collection_name].ensure_index("semantic_map_name")
    print "Created indices."
    print "done"
