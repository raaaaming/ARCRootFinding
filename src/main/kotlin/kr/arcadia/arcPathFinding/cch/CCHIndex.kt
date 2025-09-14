package kr.arcadia.arcPathFinding.cch

class CCHIndex(
    val rank: IntArray,      // node -> rank (작을수록 먼저 수축)
    val upOff: IntArray,     // Upward CSR
    val upTo: IntArray,      // Upward neighbors (dest rank가 더 큼)
    val upMid: IntArray,     // 쇼트컷이면 생성 시점의 '중간 노드 v' (원래 간선이면 -1)
    val upW: IntArray        // (선택) 가중치 저장 공간. 커스터마이즈 단계에서 채움.
) {
    val edgeCount get() = upTo.size
}