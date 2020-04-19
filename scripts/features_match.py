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
import threading
import sys, getopt


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


def match(thr, pathFeatures):
    with open('params.json') as jsonFile:
        paramsJson = json.load(jsonFile)

    # Get params from json
    exe = paramsJson['feature_match']['exe']
    pathOutput = paramsJson['feature_match']['output']['path']
    pathCombinations = paramsJson['feature_match']['features']['path']
    # listCombinations = os.listdir(pathCombinations)
    listCombinations = pathFeatures.split(',')

    if not os.path.exists(pathOutput):
        os.makedirs(pathOutput)

    count = 0
    for combination in listCombinations:
        pathCombination = '{}/{}'.format(pathCombinations, combination)

        # creating fileout
        filePathOutput = '{}/{}_{}.dat'.format(pathOutput, combination,
                                               '_'.join(thr.split(',')))
        # fill header
        with open(filePathOutput, 'w') as fileoutput:
            fileoutput.write(
                'dist,distN,distN2,media,mediana,corr,matches,s_subject,s_tp,s_exp,s_sample,t_subject,t_tp,t_exp,t_sample\n'
            )
        fileoutput.close

        for fileFeature in os.listdir(pathCombination):
            labelSource = fileFeature.split('-')[0].split('_')
            outSource = ','.join(labelSource)

            ## Neutral vs Neutral
            if (labelSource[1] != 'N') or (labelSource[3] != '0'):
                # if labelSource[0] == 'bs071' and labelSource[1] == 'O':
                #     featureSource = '{}/{}'.format(pathCombination, fileFeature)

                list_thread = []
                for i in range(0, 105):
                    thread_corr = threading.Thread(target=correspondence_match,
                                                   args=(
                                                       i,
                                                       labelSource,
                                                       pathCombination,
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
                print('feito para: {} - {}% das neutras analizadas\n'.format(
                    fileFeature, round(100 * count / 575, 2)))

        #         break
        # break


def mainArgs(argv):

    try:
        opts, args = getopt.getopt(argv, "ht:f:", ["threshold=", "folder="])
    except getopt.GetoptError:
        print('feature_match.py -t <threshold_params> -f <folder>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('feature_match.py -t <threshold_params> -f <folder>')
            sys.exit()
        elif opt in ("-t", "--threshold"):
            thr = arg
        elif opt in ("-f", "--folder"):
            pathFeatures = arg

    match(thr, pathFeatures)


if __name__ == "__main__":
    mainArgs(sys.argv[1:])
