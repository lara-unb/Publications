#include "SonarFilter.hpp"

//#include "utils/Logger.hpp"
#include <qi/log.hpp>
#include <limits>

SonarFilter::SonarFilter(){


   std::vector<int> left;
   std::vector<int> middle;
   std::vector<int> right;
   sonarFiltered.push_back(left);
   sonarFiltered.push_back(middle);
   sonarFiltered.push_back(right);

}


/* Update the filtered observations */
void SonarFilter::update(  std::vector< std::vector<int> > sonarWindow){
   qiLogDebug("Motion") << "Processing sonar right\n";
   sonarFiltered[Sonar::LEFT] = cluster(sonarWindow[Sonar::LEFT], MERGE_CUTOFF, CONFIDENCE_CUTOFF);
   qiLogDebug("Motion") << "Processing sonar middle\n";
   sonarFiltered[Sonar::MIDDLE] = cluster(sonarWindow[Sonar::MIDDLE], MERGE_CUTOFF, CONFIDENCE_CUTOFF);
   qiLogDebug("Motion") << "Processing sonar left\n";
   sonarFiltered[Sonar::RIGHT] = cluster(sonarWindow[Sonar::RIGHT], MERGE_CUTOFF, CONFIDENCE_CUTOFF); 

}


std::vector<int> SonarFilter::cluster(std::vector<int> &sonar, int merge_cutoff, int confidence_cutoff){

   std::sort(sonar.begin(), sonar.end());
   std::vector<int> result;
   
   if (sonar.size()==0) return result;

   int sum = sonar[0];
   int count = 1;
   int mean = sonar[0];

   qiLogDebug("Motion") << "1st Mean: " << mean << ", ";

   int i=1;
   while (i< (int) sonar.size()){
         
      if ( std::abs(sonar[i] - mean) < merge_cutoff){ // merge into cluster
         qiLogDebug("Motion") << "merging with: " << sonar[i];
         count++; 
         sum += sonar[i];
         mean = sum/count;
         qiLogDebug("Motion") << ", new mean: " << mean << "\n";
      } else { // save and start a new cluster
         qiLogDebug("Motion") << "separate from: " << sonar[i];
         if (count >= confidence_cutoff || mean < 0.35f){
            result.push_back(mean);
            qiLogDebug("Motion") << ", saving mean: " << mean;
         }
         sum = sonar[i];
         count = 1;
         mean = sonar[i];
         qiLogDebug("Motion") << ", new mean: " << mean << "\n";
      }
      i++;
   }
   if (count >= confidence_cutoff || mean < 0.35f){
      result.push_back(mean);
      qiLogDebug("Motion") << "Saving mean: " << mean << "\n";
   }   
   return result;
}


