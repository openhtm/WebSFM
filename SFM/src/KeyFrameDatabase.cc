/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "KeyFrameDatabase.h"

#include <mutex>

#include "KeyFrame.h"
#include "ORBVocabulary.h"
// #include "DBoW2/DBoW2/BowVector.h"
#include <fbow/fbow.h>


namespace ORB_SLAM2 {

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary& voc)
    : orb_vocabulary_(&voc) {
  // inverted_files_.resize(voc.size());
}

void KeyFrameDatabase::add(KeyFrame* keyframe) {
  std::lock_guard<std::mutex> lock(mutex_);
  for(auto it : keyframe->bow_vector_) {
    inverted_files_[it.first].push_back(keyframe);
  }
  // for (DBoW2::BowVector::const_iterator vit = keyframe->bow_vector_.begin(),
  //                                       vend = keyframe->bow_vector_.end();
  //      vit != vend; vit++)
  //   inverted_files_[vit->first].push_back(keyframe);
}

void KeyFrameDatabase::erase(KeyFrame* keyframe) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Erase elements in the Inverse File for the entry
  for(auto it : keyframe->bow_vector_) {
    if(!static_cast<bool>(inverted_files_.count(it.first)))
      continue;

    std::list<KeyFrame*>& keyframes = inverted_files_.at(it.first);
    for(auto it2 = keyframes.begin(), lend = keyframes.end(); it2 != lend; it2++) {
      if(*it2 == keyframe) {
        keyframes.erase(it2);
        break;
      }
    }
  }
  // for (DBoW2::BowVector::const_iterator vit = keyframe->bow_vector_.begin(),
  //                                       vend = keyframe->bow_vector_.end();
  //      vit != vend; vit++) {
  //   // List of keyframes that share the word
  //   std::list<KeyFrame*>& keyframes = inverted_files_[vit->first];

  //   for (std::list<KeyFrame*>::iterator lit = keyframes.begin(),
  //                                       lend = keyframes.end();
  //        lit != lend; lit++) {
  //     if (keyframe == *lit) {
  //       keyframes.erase(lit);
  //       break;
  //     }
  //   }
  // }
}

void KeyFrameDatabase::clear() {
  inverted_files_.clear();
}

std::vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(
    KeyFrame* keyframe, float minScore) {
  std::set<KeyFrame*> connected_keyframes = keyframe->GetConnectedKeyFrames();
  std::list<KeyFrame*> keyframes_sharing_words;

  // Search all keyframes that share a word with current keyframes
  // Discard keyframes connected to the query keyframe
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for(auto it : keyframe->bow_vector_) {
      if(!static_cast<bool>(inverted_files_.count(it.first)))
        continue;

      std::list<KeyFrame*>& keyframes = inverted_files_.at(it.first);
      for(auto it2 = keyframes.begin(); it2 != keyframes.end(); it2++) {
        KeyFrame* i_kf = *it2;
        if (i_kf->n_loop_query_ != keyframe->id_) {
          i_kf->n_loop_words_ = 0;
          if (!connected_keyframes.count(i_kf)) {
            i_kf->n_loop_query_ = keyframe->id_;
            keyframes_sharing_words.push_back(i_kf);
          }
        }
        i_kf->n_loop_words_++;
      }
    }
    // for (DBoW2::BowVector::const_iterator vit = keyframe->bow_vector_.begin(),
    //                                       vend = keyframe->bow_vector_.end();
    //      vit != vend; vit++) {
    //   std::list<KeyFrame*>& keyframes = inverted_files_[vit->first];

    //   for (auto lit = keyframes.begin(); lit != keyframes.end(); lit++) {
    //     KeyFrame* i_kf = *lit;
    //     if (i_kf->n_loop_query_ != keyframe->id_) {
    //       i_kf->n_loop_words_ = 0;
    //       if (!connected_keyframes.count(i_kf)) {
    //         i_kf->n_loop_query_ = keyframe->id_;
    //         keyframes_sharing_words.push_back(i_kf);
    //       }
    //     }
    //     i_kf->n_loop_words_++;
    //   }
    // }
  }

  if (keyframes_sharing_words.empty()) {
    return std::vector<KeyFrame*>();
  }

  std::list<std::pair<float, KeyFrame*> > score_and_matches;

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (std::list<KeyFrame*>::iterator lit = keyframes_sharing_words.begin(),
                                 lend = keyframes_sharing_words.end();
       lit != lend; lit++) {
    if ((*lit)->n_loop_words_ > maxCommonWords)
      maxCommonWords = (*lit)->n_loop_words_;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  int nscores = 0;

  // Compute similarity score. Retain the matches whose score is higher than
  // minScore
  for(auto it : keyframe->bow_vector_) {
    if(!static_cast<bool>(inverted_files_.count(it.first)))
      continue;

    std::list<KeyFrame*>& keyframes = inverted_files_.at(it.first);
    for(auto it2 = keyframes.begin(); it2 != keyframes.end(); it2++) {
      KeyFrame* i_kf = *it2;
      if (i_kf->n_loop_words_ > minCommonWords) {
        nscores++;
        float score = fbow::fBow::score(keyframe->bow_vector_, i_kf->bow_vector_);
        // orb_vocabulary_->score(keyframe->bow_vector_, i_kf->bow_vector_);
        i_kf->loop_score_ = score;
        if (score >= minScore)
          score_and_matches.push_back(std::make_pair(score, i_kf));
      }
    }
  }
  // for (std::list<KeyFrame*>::iterator lit = keyframes_sharing_words.begin(),
  //                                lend = keyframes_sharing_words.end();
  //      lit != lend; lit++) {
  //   KeyFrame* i_kf = *lit;

  //   if (i_kf->n_loop_words_ > minCommonWords) {
  //     nscores++;

  //     float score =
  //         orb_vocabulary_->score(keyframe->bow_vector_, i_kf->bow_vector_);

  //     i_kf->loop_score_ = score;
  //     if (score >= minScore)
  //       score_and_matches.push_back(std::make_pair(score, i_kf));
  //   }
  // }

  if (score_and_matches.empty()) {
    return std::vector<KeyFrame*>();
  }

  std::list<std::pair<float, KeyFrame*> > acc_score_and_matches;
  float bestAccScore = minScore;

  // Lets now accumulate score by covisibility
  for (auto it : score_and_matches) {
    KeyFrame* i_kf = it.second;
    std::vector<KeyFrame*> neighs = i_kf->GetBestCovisibilityKeyFrames(10);

    float bestScore = it.first;
    float accScore = it.first;
    KeyFrame* best_keyframe = i_kf;
    for(auto it2 : neighs) {
      KeyFrame* j_kf = it2;
      if (j_kf->n_loop_query_ == keyframe->id_ &&
          j_kf->n_loop_words_ > minCommonWords) {
        accScore += j_kf->loop_score_;
        if (j_kf->loop_score_ > bestScore) {
          best_keyframe = j_kf;
          bestScore = j_kf->loop_score_;
        }
      }
    }
    // for (std::vector<KeyFrame*>::iterator vit = neighs.begin(),
    //                                       vend = neighs.end();
    //      vit != vend; vit++) {
    //   KeyFrame* j_kf = *vit;
    //   if (j_kf->n_loop_query_ == keyframe->id_ &&
    //       j_kf->n_loop_words_ > minCommonWords) {
    //     accScore += j_kf->loop_score_;
    //     if (j_kf->loop_score_ > bestScore) {
    //       best_keyframe = j_kf;
    //       bestScore = j_kf->loop_score_;
    //     }
    //   }
    // }

    acc_score_and_matches.push_back(std::make_pair(accScore, best_keyframe));
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;

  std::set<KeyFrame*> alread_added_keyframes;
  std::vector<KeyFrame*> loop_candidates_;
  loop_candidates_.reserve(acc_score_and_matches.size());

  for (auto it : acc_score_and_matches) {
    if (it.first > minScoreToRetain) {
      KeyFrame* i_kf = it.second;
      if (!alread_added_keyframes.count(i_kf)) {
        loop_candidates_.push_back(i_kf);
        alread_added_keyframes.insert(i_kf);
      }
    }
  }

  return loop_candidates_;
}

std::vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(
    Frame* frame) {
  std::list<KeyFrame*> keyframes_sharing_words;

  // Search all keyframes that share a word with current frame
  {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto it : frame->bow_vector_) {
      if(!static_cast<bool>(inverted_files_.count(it.first)))
        continue;

      std::list<KeyFrame*>& keyframes = inverted_files_.at(it.first);
      for(auto it2 : keyframes) {
        KeyFrame* i_kf = it2;
        if (i_kf->reloc_query_ != frame->id_) {
          i_kf->n_reloc_words_ = 0;
          i_kf->reloc_query_ = frame->id_;
          keyframes_sharing_words.push_back(i_kf);
        }
        i_kf->n_reloc_words_++;
      }
      // for (std::list<KeyFrame*>::iterator lit = keyframes.begin(),
      //                                     lend = keyframes.end();
      //      lit != lend; lit++) {
      //   KeyFrame* i_kf = *lit;
      //   if (i_kf->reloc_query_ != frame->id_) {
      //     i_kf->n_reloc_words_ = 0;
      //     i_kf->reloc_query_ = frame->id_;
      //     keyframes_sharing_words.push_back(i_kf);
      //   }
      //   i_kf->n_reloc_words_++;
      // }
    }
  }
  if (keyframes_sharing_words.empty()) {
    return std::vector<KeyFrame*>();
  }

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (auto it : keyframes_sharing_words) {
    if (it->n_reloc_words_ > maxCommonWords)
      maxCommonWords = it->n_reloc_words_;
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<std::pair<float, KeyFrame*> > score_and_matches;

  int nscores = 0;

  // Compute similarity score.
  for (auto it : keyframes_sharing_words) {
    KeyFrame* i_kf = it;
    if (i_kf->n_reloc_words_ > minCommonWords) {
      nscores++;
      float score = fbow::fBow::score(frame->bow_vector_, i_kf->bow_vector_);
          // orb_vocabulary_->score(frame->bow_vector_, i_kf->bow_vector_);
      i_kf->reloc_score_ = score;
      score_and_matches.push_back(std::make_pair(score, i_kf));
    }
  }

  if (score_and_matches.empty()) {
    return std::vector<KeyFrame*>();
  }

  std::list<std::pair<float, KeyFrame*> > acc_score_and_matches;
  float bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (auto it : score_and_matches) {
    KeyFrame* i_kf = it.second;
    std::vector<KeyFrame*> neighs = i_kf->GetBestCovisibilityKeyFrames(10);

    float bestScore = it.first;
    float accScore = bestScore;
    KeyFrame* best_keyframe = i_kf;

    for(auto j_kf : neighs) {
      if (j_kf->reloc_query_ != frame->id_) continue;

      accScore += j_kf->reloc_score_;
      if (j_kf->reloc_score_ > bestScore) {
        best_keyframe = j_kf;
        bestScore = j_kf->reloc_score_;
      }
    }
    // for (vector<KeyFrame*>::iterator vit = neighs.begin(), vend = neighs.end();
    //      vit != vend; vit++) {
    //   KeyFrame* j_kf = *vit;
    //   if (j_kf->reloc_query_ != frame->id_) continue;

    //   accScore += j_kf->reloc_score_;
    //   if (j_kf->reloc_score_ > bestScore) {
    //     best_keyframe = j_kf;
    //     bestScore = j_kf->reloc_score_;
    //   }
    // }
    acc_score_and_matches.push_back(std::make_pair(accScore, best_keyframe));
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;
  std::set<KeyFrame*> alread_added_keyframes;
  std::vector<KeyFrame*> reloc_candidates;
  reloc_candidates.reserve(acc_score_and_matches.size());

  for (auto it : acc_score_and_matches) {
    const float& score = it.first;
    if (score > minScoreToRetain) {
      KeyFrame* i_kf = it.second;
      if (!alread_added_keyframes.count(i_kf)) {
        reloc_candidates.push_back(i_kf);
        alread_added_keyframes.insert(i_kf);
      }
    }
  }

  return reloc_candidates;
}
}  // namespace ORB_SLAM2
