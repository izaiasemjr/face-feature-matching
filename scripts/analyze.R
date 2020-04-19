
## FUnctions --------------------------------------------------------
comparison <- function(pred) {
  v1 = toString(pred$s_subject)
  v2 = toString(pred$t_subject)
  if(v1==v2){
    return (TRUE)
  }
  else return (FALSE)
}
## End Functions ----------------------------------------------------

# setwd(dir = "Documentos/MESTRADO/teste-features/face-feature-matching/scripts/")
# m = read.csv("./results_match/FPFH/feat_30_norm_15_key_sift_normal_2_3_4_0.001_th_fixe_450.dat")
m = read.csv("./results_match/FPFH/feat_30_norm_10_key_sift_normal_2_3_4_0.001_th_k_5.dat")
# m = read.csv("./results_match/FPFH/feat_30_norm_10_key_sift_normal_2_3_4_0.001_th_median_0.5.dat")

## NEUTRAL  NN0
# X_test =  m[m$s_tp=='N' & m$s_sample!='0',]
# X_test =  m[m$s_tp=='O' & m$s_exp=='GLASSES', ]
X_test =  m[m$s_tp=='O' & m$s_subject== 'bs057' & m$s_exp=='HAIR' , ]
X_test =  m[m$s_tp=='N' & m$s_subject== 'bs057' & m$s_sample== '3'  , ]


subjects=unique(X_test$s_subject)
y_test <- c()
y_pred <- c
accuracy=c(0,0,0,0)
scores=c(0,0,0,0)
i=0
for (subject in subjects){ 
  i=i+1
  
  # get table comparisons for each subject
  M_subject = X_test[X_test$s_subject == subject ,]

  # predict subject on target using min distance
  pred_d = M_subject[M_subject$dist==min(M_subject$dist),]
  # predict subject on target using min distance
  pred_d2 = M_subject[M_subject$distN2==min(M_subject$distN2),]
  # predict subject on target using min distances average
  pred_a = M_subject[M_subject$media==min(M_subject$media),]
  # predict subject on target using median distances
  pred_m = M_subject[M_subject$mediana==min(M_subject$mediana),]
    
  if(comparison(pred_d)){
    accuracy[[1]] =accuracy[[1]]+1
    scores[[1]] = pred_d$dist
  }
  if(comparison(pred_d2)){
    accuracy[[2]] =accuracy[[2]]+1
    scores[[2]] = pred_d$distN2
  }
  if(comparison(pred_a)){
    accuracy[[3]] =accuracy[[3]]+1
    scores[[3]] = pred_d$media
  }
  if(comparison(pred_m)){
    accuracy[[4]] =accuracy[[4]]+1
    scores[[4]] = pred_d$mediana
  }  

}

accuracy=100*accuracy/i
print((accuracy))
print(scores)





