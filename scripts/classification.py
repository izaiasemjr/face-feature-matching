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
import argparse
import threading


def extraction(feature_radius, normal_radius):
    with open('params.json') as jsonFile:
        paramsJson = json.load(jsonFile)

    # Get params from json
    exe = paramsJson['feature_extraction']['exe']
    pathPeople = paramsJson['feature_extraction']['clouds']['path']
    keypointsPath = paramsJson['feature_extraction']['keypoints']['path']
    pathOutput = paramsJson['feature_extraction']['output']['path']
    # verify if path exits
    if not os.path.exists(pathOutput):
        os.makedirs(pathOutput)

    # Separete just neutral faces and aply feature extraction
    command = ""
    count = 0
    for person in os.listdir(pathPeople):
        if person.startswith("."): continue
        pathPerson = pathPeople + "/" + person
        for cloud in os.listdir(pathPerson):
            if (cloud.split('.')[1] != 'pcd'): continue
            tp = cloud.split('.')[0].split('_')[1]
            if tp == 'N' or tp == 'O':
                # cloudSplit = cloud.split('.')[0].split('_')
                # if (cloudSplit[0] == 'bs071' or
                #     (cloudSplit[1] == 'N' and cloudSplit[3] == '0')):

                cloud_param = "-cloud {}/{}/{} ".format(
                    pathPeople, person, cloud)

                keypoints_params = "-keypoints from_file,{}/{}/{}".format(
                    keypointsPath, person, cloud)

                feature_params = "-features fpfh,{} ".format(feature_radius)

                normal_params = "-normal {} ".format(normal_radius)

                output_param = "-output {}/{}-desc.pcd".format(
                    pathOutput,
                    cloud.split('.')[0])

                command = "{} {} {} {} {} {} -debug 0".format(
                    exe, cloud_param, normal_params, keypoints_params,
                    feature_params, output_param)

                os.system(command)
                # print(command)

                # debug
                count = count + 1
                percent = round(100 * count / 680, 2)
                print(
                    '{}% concluído -- feito para {} - configs extraction: feat:{}, norm:{} \n'
                    .format(percent, cloud, feature_radius, normal_radius))


def correspondence_match(idx_person, labelSource, pathCombination,
                         featureSource, filePathOutput, outSource, thr, exe):
    labelTarget = 'bs{}_{}_{}_{}'.format(
        str(idx_person).zfill(3), 'N', 'N', '0')
    outTarget = ','.join(labelTarget.split('_'))

    featureTarget = '{}/{}-desc.pcd'.format(pathCombination, labelTarget)

    features_params = '-features {},{}'.format(featureSource, featureTarget)

    output_params = '-output {}:{}:{}'.format(filePathOutput, outSource,
                                              outTarget)
    threshold_params = '-th {}'.format(thr)

    command = '{} {} {} {} -debug 0'.format(exe, features_params,
                                            output_params, threshold_params)

    os.system(command)


def match(feature_radius, normal_radius, thr):
    with open('params.json') as jsonFile:
        paramsJson = json.load(jsonFile)

    # Get params from json
    exe = paramsJson['feature_match']['exe']
    pathOutput = paramsJson['feature_match']['output']['path']
    pathFeatures = paramsJson['feature_extraction']['output']['path']

    if not os.path.exists(pathOutput):
        os.makedirs(pathOutput)

    count = 0
    # creating fileout
    filePathOutput = '{}/{}_{}_{}.dat'.format(pathOutput, feature_radius,
                                              normal_radius, thr)
    # fill header
    with open(filePathOutput, 'w') as fileoutput:
        fileoutput.write(
            'dist,distN,distN2,media,mediana,corr,matches,s_subject,s_tp,s_exp,s_sample,t_subject,t_tp,t_exp,t_sample\n'
        )
    fileoutput.close

    for fileFeature in os.listdir(pathFeatures):
        labelSource = fileFeature.split('-')[0].split('_')
        outSource = ','.join(labelSource)

        ## Neutral vs Neutral
        if (labelSource[1] != 'N') or (labelSource[3] != '0'):
            # if labelSource[0] == 'bs071' and labelSource[1] == 'O':
            featureSource = '{}/{}'.format(pathFeatures, fileFeature)

            list_thread = []
            for i in range(0, 105):
                thread_corr = threading.Thread(target=correspondence_match,
                                               args=(
                                                   i,
                                                   labelSource,
                                                   pathFeatures,
                                                   featureSource,
                                                   filePathOutput,
                                                   outSource,
                                                   thr,
                                                   exe,
                                               ))

                list_thread.append(thread_corr)
                thread_corr.start()

            for th in list_thread:
                th.join()

            count = count + 1
            percent = round(100 * count / 575, 2)
            print(
                ' {}% concluido -- feito para: {} -- configs match -> feat:{}, norm:{}, th: {} \n'
                .format(percent, fileFeature, feature_radius, normal_radius,
                        thr))

    #         break
    # break


if __name__ == "__main__":
    feature_radius = 25
    normal_radius = 10
    threshold = 'th_k,{}'.format(10)

    # extraction(feature_radius, normal_radius),
    print("-------------------------------------")
    match(feature_radius, normal_radius, threshold)
