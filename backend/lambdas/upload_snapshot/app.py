import os
import json
import boto3
import datetime


def lambda_handler(event, context):
    bucketName = os.environ['DataBucket']

    body = json.loads(event['body'])

    deviceId = body['deviceId']
    sessionId = body['sessionId']

    baseFileName = (datetime.datetime.now()).strftime('%Y-%m-%dT%H:%M:%S')

    key = f'{deviceId}/{sessionId}/{baseFileName}.txt'

    s3 = boto3.client('s3')

    params = {'Bucket': bucketName, 'Key': key}
    print(params);

    preSignedUrl = s3.generate_presigned_url('put_object', Params=params, HttpMethod='PUT')

    return {
        'statusCode': 200,
        'body': json.dumps({
            'preSignedUrl': preSignedUrl,
        }),
    }

