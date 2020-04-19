

# open file
faces = read.csv("./results_match/FPFH/feat_30_norm_10_key_sift_normal_2_3_4_0.001_th_k_10.dat")

# id attribute
facesId= cbind(data, "fid" = paste(data$s_subject,data$s_tp,data$s_exp,data$s_sample,sep='_'))
# filter for experiments
facesId = facesId[facesId$s_tp=='N',]


# iterate over each sample face 
accuracies = c(0,0,0,0,0)
facesUnique = unique(facesId$fid)
for (faceId in facesUnique) {
  
  face = (facesId[facesId$fid == faceId,])
  
  # prediction by dist
  pred_dist = (face[face$dist==min(face$dist),  c('s_subject','t_subject')])
  if(as.character(pred_dist$s_subject) == as.character(pred_dist$t_subject)){
    accuracies[1] = accuracies[1] +1
    }
  
  # prediction by dist/N
  pred_distN = (face[face$distN==min(face$distN),  c('s_subject','t_subject')])
  if(as.character(pred_distN$s_subject) == as.character(pred_distN$t_subject)){
    accuracies[2] = accuracies[2] +1
  }
  # prediction by dist/N2
  pred_distN2 = (face[face$distN2==min(face$distN2),  c('s_subject','t_subject')])
  if(as.character(pred_distN2$s_subject) == as.character(pred_distN2$t_subject)){
    accuracies[2] = accuracies[3] +1
  }
  
  
}
 print(accuracies/length(facesUnique))