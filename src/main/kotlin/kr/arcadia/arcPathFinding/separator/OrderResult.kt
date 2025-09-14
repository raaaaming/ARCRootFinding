package kr.arcadia.arcPathFinding.separator

data class OrderResult(

    val order: IntArray, // rank -> nodeId (수축 순서)

    val level: IntArray, // nodeId -> level (분할 깊이; 루트=0, 깊어질수록 +1)

)