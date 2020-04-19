#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  3 23:18:24 2020

@author: izaiasjr
"""
import os
import json
import shutil
import numpy as np
import sys, getopt


def extraction(key_method):
    with open('params.json') as jsonFile:
        paramsJson = json.load(jsonFile)

    # iterations
    count_global = 0

    # Get params from json
    exe = paramsJson['feature_extraction']['exe']
    pathPeople = paramsJson['feature_extraction']['clouds']['path']
    keypointsPath = paramsJson['feature_extraction']['keypoints']['from_file'][
        'path']
    pathOutput = paramsJson['feature_extraction']['output']['path']

    # params
    # normals_rays = paramsJson["feature_extraction"]["normals"]["omp"][
    #     "radiusSearch"]
    # features_rays = paramsJson["feature_extraction"]["features"]["FPFH"][
    #     "radiusSearch"]

    features_rays = [30]
    normals_rays = [10]

    for feature_radius in features_rays:
        for normal_radius in normals_rays:

            # creat path if not exist (by region)
            pathOutputFinal = '{}/feat_{}_norm_{}_key_{}'.format(
                pathOutput,
                feature_radius,
                normal_radius,
                ('_').join(key_method.split(',')),
            )
            # verify if path exits
            if not os.path.exists(pathOutputFinal):
                os.makedirs(pathOutputFinal)

            # Separete just neutral faces and aply feature extraction
            command = ""
            count = 0
            for person in os.listdir(pathPeople):
                if not person.startswith("."):
                    pathPerson = pathPeople + "/" + person
                    for cloud in os.listdir(pathPerson):
                        # tp = cloud.split('.')[0].split('_')[1]
                        # if tp == 'N' or tp == 'O':
                        cloudSplit = cloud.split('.')[0].split('_')
                        if (cloudSplit[0] == 'bs071' or
                            (cloudSplit[1] == 'N' and cloudSplit[3] == '0')):

                            cloud_param = "-cloud {}/{}/{} ".format(
                                pathPeople, person, cloud)

                            normal_params = "-normal {} ".format(normal_radius)

                            if (key_method == 'from_file'):
                                keypoints_params = "-keypoints from_file,{}/{}/{}".format(
                                    keypointsPath, person, cloud)
                            else:
                                keypoints_params = "-keypoints {}".format(
                                    key_method)

                            feature_params = "-features fpfh,{} ".format(
                                feature_radius)

                            output_param = "-output {}/{}-desc.pcd".format(
                                pathOutputFinal,
                                cloud.split('.')[0])

                            command = "{} {} {} {} {} {} -debug 0".format(
                                exe, cloud_param, normal_params,
                                keypoints_params, feature_params, output_param)

                            os.system(command)
                            # print(command)
                            count_global = count_global + 1
                            total = 680 * len(normals_rays) * len(
                                features_rays)
                            print(
                                'feito para {} - {}% concluído \noutput: {}\n'.
                                format(cloud,
                                       round(100 * count_global / total, 2),
                                       output_param))


def mainArgs(argv):

    try:
        opts, args = getopt.getopt(argv, "hk:", ["key="])
    except getopt.GetoptError:
        print(
            'Invalid arguments, Its should be like:\nfeatures_extraction.py -k <key_methode> \n'
        )
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print(
                'Try something like: \nfeatures_extraction.py -k <key_methode> \n'
            )
            sys.exit()
        elif opt in ("-k", "--key"):
            key_method = arg

    extraction(key_method)


if __name__ == "__main__":
    mainArgs(sys.argv[1:])
