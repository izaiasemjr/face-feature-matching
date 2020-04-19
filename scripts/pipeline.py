import os

## config 1  (sift)
features = 'python3 features_extraction.py -k sift_normal,2,3,4,0.001'
match1 = 'python3 features_match.py  -t  th_k,10  -f feat_30_norm_10_key_sift_normal_2_3_4_0.001'
match2 = 'python3 features_match.py  -t  th_median,0.5  -f feat_30_norm_10_key_sift_normal_2_3_4_0.001'
match3 = 'python3 features_match.py  -t  th_fixe,440  -f feat_30_norm_10_key_sift_normal_2_3_4_0.001'
os.system(features)
os.system(match1)
os.system(match2)
os.system(match3)

## config 2
features = 'python3 features_extraction.py -k from_file'
match1 = 'python3 features_match.py  -t  th_fixe,440  -f feat_30_norm_10_key_from_file'
os.system(features)
os.system(match1)

# alarm='c=10; while[$c -ge 0]; do spd-say Your code is done!; sleep 3; let c--; done'
# os.system(alarm)
msg = "curl 'https://api.twilio.com/2010-04-01/Accounts/AC51d7f44d42fa4b48a1f28fe15741a5d5/Messages.json' -X POST \
--data-urlencode 'To=whatsapp:+558597985448' \
--data-urlencode 'From=whatsapp:+14155238886' \
--data-urlencode 'Body=Seu codigo terminou' \
-u AC51d7f44d42fa4b48a1f28fe15741a5d5:599bc6a71f04b6b01a599e38b6274f66"

os.system(msg)