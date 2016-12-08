#!/usr/bin/env python

from pymongo import MongoClient
import csv
import argparse
import sys


def get_dialect(filename):
    with open(filename, 'rb') as csvfile:
        return csv.Sniffer().sniff(csvfile.read(1024), delimiters=";,")

def file_reader(inputfile):
    print "openning %s" % inputfile
    dialect = get_dialect(filename=inputfile)
    with open(inputfile) as csvfile:
        reader = csv.DictReader(csvfile, dialect=dialect)
        for row in reader:
            yield row


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
    for row in file_reader(args.input):
        row["semantic_map_name"] = args.dataset_name
        db[args.collection_name].insert(row)
    db[args.collection_name].ensure_index("shop_id")
    db[args.collection_name].ensure_index("semantic_map_name")
    print "done"
