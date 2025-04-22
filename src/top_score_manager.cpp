#include "top_score_manager.h"
#include <fstream>
#include <iostream>

TopScoreManager::TopScoreManager(const std::string& filename)
  : _filename(filename)
{}

int TopScoreManager::LoadTopScore() {
  std::ifstream in(_filename);
  if (!in.is_open()) {
    // No file yet → top score is zero.
    _cachedTopScore = 0;
    return 0;
  }

  int score = 0;
  if (in >> score) {
    _cachedTopScore = score;
  } else {
    // Malformed file → ignore and start at zero.
    std::cerr << "Warning: could not parse top score in " << _filename << "\n";
    _cachedTopScore = 0;
  }
  return _cachedTopScore;
}

bool TopScoreManager::SaveTopScore(int score) {
  if (score <= _cachedTopScore) {
    // Not a new record.
    return false;
  }

  std::ofstream out(_filename, std::ios::trunc);
  if (!out.is_open()) {
    std::cerr << "Error: could not open " << _filename << " for writing\n";
    return false;
  }

  out << score << "\n";
  if (!out) {
    std::cerr << "Error: failed to write top score\n";
    return false;
  }

  _cachedTopScore = score;
  return true;
}
